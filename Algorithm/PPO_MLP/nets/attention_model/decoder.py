import torch
from torch import nn
import torch.nn.functional as F

from .context import AutoContext
from .dynamic_embedding import AutoDynamicEmbedding
from .multi_head_attention import (
    AttentionScore,
    MultiHeadAttention,
)

class Decoder(nn.Module):
    def __init__(self, embedding_dim, step_context_dim, n_heads, tanh_clipping, time_levels):
        super(Decoder, self).__init__()
        self.embedding_dim = embedding_dim
        
        # 基础映射
        self.project_node_embeddings = nn.Linear(embedding_dim, 3 * embedding_dim, bias=False)
        self.project_fixed_context = nn.Linear(embedding_dim, embedding_dim, bias=False)
        self.project_step_context = nn.Linear(step_context_dim, embedding_dim, bias=False)

        # # 时间层级 Embedding：由于节点和时间是不同维度的决策，需要专门的时间向量
        # self.time_embeddings = nn.Parameter(torch.Tensor(time_levels, embedding_dim))
        # self.time_embeddings.data.uniform_(-1, 1)

        # 时间选择专用：轻量级 MLP Head
        # 将融合后的上下文直接映射到时间层级维度
        self.time_head = nn.Sequential(
            nn.Linear(embedding_dim, embedding_dim // 2),
            nn.ReLU(),
            nn.Linear(embedding_dim // 2, time_levels)
        )

        self.context = AutoContext({"context_dim": step_context_dim})
        self.dynamic_embedding = AutoDynamicEmbedding(
            {"embedding_dim": embedding_dim}
        )
        self.glimpse = MultiHeadAttention(embedding_dim=embedding_dim, n_heads=n_heads)
        self.pointer = AttentionScore(use_tanh=True, C=tanh_clipping)

        self.decode_type = None

    def set_decode_type(self, decode_type):
        self.decode_type = decode_type

    def _precompute(self, embeddings):
        graph_embed = embeddings.mean(1)
        graph_context = self.project_fixed_context(graph_embed).unsqueeze(-2)
        
        # 节点部分的 K, V, Logit_K
        glimpse_key, glimpse_val, logit_key = self.project_node_embeddings(embeddings).chunk(3, dim=-1)

        return (embeddings, graph_context, glimpse_key, glimpse_val, logit_key)

    def advance(self, cached_embeddings, state, selected_node=None):
        """
        分步解码：
        1. selected_node is None -> 计算节点 Logits
        2. selected_node is Batch Tensor -> 计算时间 Logits
        """
        node_embeddings, graph_context, glimpse_K, glimpse_V, logit_K = cached_embeddings
        batch_size = node_embeddings.size(0)

        # 1. 计算基础上下文
        context = self.context(node_embeddings, state) # [batch, 1, step_context_dim]
        
        # 2. 如果已选定节点，将选定节点的 Embedding 融合进上下文
        if selected_node is not None:
            # 提取选定节点的 Embedding: [batch, 1, embed_dim]
            selected_embed = node_embeddings.gather(
                1, selected_node.view(-1, 1, 1).expand(-1, 1, self.embedding_dim)
            )
            # 这里简单采用相加，也可以改用 concat + linear （简单相加效果已经不错了）
            step_context = self.project_step_context(context) + selected_embed
        else:
            step_context = self.project_step_context(context)

        query = graph_context + step_context  # [batch, 1, embed_dim]

        # 3. 获取动态特征（如果有）
        glimpse_key_dynamic, glimpse_val_dynamic, logit_key_dynamic = self.dynamic_embedding(state)
        
        # 4. 获取全量掩码
        full_mask = state.get_mask() # [batch, 1, node_count + time_levels]
        node_count = node_embeddings.size(1)

        if selected_node is None:
            # --- 节点选择阶段 ---
            cur_glimpse_K = glimpse_K + glimpse_key_dynamic
            cur_glimpse_V = glimpse_V + glimpse_val_dynamic
            cur_logit_K = logit_K + logit_key_dynamic
            cur_mask = full_mask[:, :, :node_count]
            
            logits, glimpse = self.calc_logits(query, cur_glimpse_K, cur_glimpse_V, cur_logit_K, cur_mask)
        else:
            # # --- 时间选择阶段 ---
            # # 使用预定义的时间 Embedding
            # t_embeddings = self.time_embeddings.unsqueeze(0).expand(batch_size, -1, -1)
            # # 为时间部分计算 K, V, Logit_K (复用节点的投影矩阵或使用独立投影)
            # t_glimpse_K, t_glimpse_V, t_logit_K = self.project_node_embeddings(t_embeddings).chunk(3, dim=-1)
            
            # cur_mask = full_mask[:, :, node_count:]
            # logits, glimpse = self.calc_logits(query, t_glimpse_K, t_glimpse_V, t_logit_K, cur_mask)

            # 直接通过 MLP 输出 Logits
            logits = self.time_head(query) # [batch, 1, time_levels]
            
            # 提取时间部分的 Mask
            cur_mask = full_mask[:, :, node_count:]
            logits = logits.masked_fill(cur_mask.bool(), float('-inf'))
            
            # 为了保持接口兼容，glimpse 返回当前查询向量
            glimpse = query

        if glimpse.dim() == 2:
            glimpse = glimpse.unsqueeze(1)

        return logits, glimpse

    def calc_logits(self, query, glimpse_K, glimpse_V, logit_K, mask):
        glimpse = self.glimpse(query, glimpse_K, glimpse_V, mask)
        logits = self.pointer(glimpse, logit_K, mask)
        return logits, glimpse