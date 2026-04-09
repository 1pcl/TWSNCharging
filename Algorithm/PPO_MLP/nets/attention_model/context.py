"""
Problem specific global embedding for global context.
"""

import torch
from torch import nn


def AutoContext(config):
    """
    Automatically select the corresponding module according to ``problem_name``
    """
    embeddingClass = UAVContext
    embedding = embeddingClass(**config)
    return embedding


def _gather_by_index(source, index):
    """
    target[i,1,:] = source[i,index[i],:]
    Inputs:
        source: [B x H x D]
        index: [B x 1] or [B]
    Outpus:
        target: [B x 1 x D]
    """
    # 确保 index 是 long 类型
    if not isinstance(index, torch.Tensor):
        index = torch.tensor(index)
    index = index.long()

    if index.dim() == 1:
        # [B] -> [B, 1, 1]
        index = index.unsqueeze(-1).unsqueeze(-1)
    elif index.dim() == 2:
        # [B, 1] -> [B, 1, 1]
        index = index.unsqueeze(-1)

    # target = torch.gather(source, 1, index.unsqueeze(-1).expand(-1, -1, source.size(-1)))
    target = torch.gather(source, 1, index.expand(-1, -1, source.size(-1)))
    return target


class PrevNodeContext(nn.Module):

    def __init__(self, context_dim):
        super(PrevNodeContext, self).__init__()
        self.context_dim = context_dim

    def _prev_node_embedding(self, embeddings, state):
        current_node = state.get_current_node()
        prev_node_embedding = _gather_by_index(embeddings, current_node)
        return prev_node_embedding

    def _state_embedding(self, embeddings, state):
        raise NotImplementedError("Please implement the embedding for your own problem.")

    def forward(self, embeddings, state):
        prev_node_embedding = self._prev_node_embedding(embeddings, state)
        state_embedding = self._state_embedding(embeddings, state)
        # Embedding of previous node + remaining capacity
        context_embedding = torch.cat((prev_node_embedding, state_embedding), -1)
        return context_embedding




class UAVContext(PrevNodeContext):

    def __init__(self, context_dim):
        super(UAVContext, self).__init__(context_dim)

    
    def _state_embedding(self, embeddings, state):
        # 1. 获取状态 [Batch, 4]
        uav_state = state.states["uav_state"]
        
        # 2. 如果是 1 维（单样本推理），先补上 Batch 维度
        if uav_state.dim() == 1:
            uav_state = uav_state.unsqueeze(0) # [4] -> [1, 4]
            
        # 3. 增加序列维度 (Time/Node dimension)，使其变为 [Batch, 1, 4]
        # 这样才能与 [Batch, 1, Embedding_Dim] 的 prev_node_embedding 拼接
        state_embedding = uav_state.unsqueeze(1) 
        
        return state_embedding

