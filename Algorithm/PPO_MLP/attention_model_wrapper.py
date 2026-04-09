import torch
from torch import nn
import torch.nn.functional as F

from nets.attention_model.attention_model import *
from sb3_contrib.common.maskable.policies import MaskableActorCriticPolicy
from sb3_contrib.common.maskable.distributions import MaskableMultiCategoricalDistribution

mydevice = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class Backbone(nn.Module):
    def __init__(self,node_dim,uav_dim ,embedding_dim=128, n_encode_layers=3, tanh_clipping=10.0, n_heads=8, time_levels=100, device="cpu"):
        super(Backbone, self).__init__()
        self.device = device
        self.embedding = AutoEmbedding({"embedding_dim": embedding_dim,"node_dim":node_dim,"uav_dim":uav_dim})

        self.encoder = GraphAttentionEncoder(
            n_heads=n_heads,
            embed_dim=embedding_dim,
            n_layers=n_encode_layers,
        )

        self.decoder = Decoder(
            embedding_dim, self.embedding.context_dim, n_heads, tanh_clipping, time_levels  
        )

    def forward(self, obs):
        state = stateWrapper(obs, device=self.device)
        input = state.states["observations"]
        
        # 1. 编码
        embedding = self.embedding(input)
        encoded_inputs, _ = self.encoder(embedding)
        cached_embeddings = self.decoder._precompute(encoded_inputs)

        # 2. 第一步：节点选择 (Node Selection)
        node_logits, _ = self.decoder.advance(cached_embeddings, state, selected_node=None)

        # # 3. 采样当前节点（为了给下一步提供上下文）
        # # 训练时使用 argmax 或根据概率分布采样，这里使用 argmax 简化逻辑
        # with torch.no_grad():
        #     selected_node = node_logits.argmax(dim=-1)

        # 3. 采样当前节点
        with torch.no_grad():
            # 使用argmax或根据logits采样
            if self.training:
                # 训练时可以使用softmax采样
                node_probs = F.softmax(node_logits.squeeze(1), dim=-1)
                selected_node = torch.multinomial(node_probs, 1).squeeze(-1)
            else:
                selected_node = node_logits.argmax(dim=-1)

        # 4. 第二步：时间选择 (Time Selection)
        time_logits, time_glimpse = self.decoder.advance(cached_embeddings, state, selected_node=selected_node)

        # 确保 logits 维度为 [batch, count]
        if node_logits.dim() == 3: node_logits = node_logits.squeeze(1)
        if time_logits.dim() == 3: time_logits = time_logits.squeeze(1)

        # 返回两组 logits 和最终的特征（用于 Critic）
        return (node_logits, time_logits), time_glimpse

class Actor(nn.Module):
    def __init__(self):
        super(Actor, self).__init__()
    
    def forward(self, x):
        logits_tuple, _ = x # 接收 (node_logits, time_logits)
        return logits_tuple

class Critic(nn.Module):
    def __init__(self, embedding_dim):
        super(Critic, self).__init__()
        self.mlp = nn.Sequential(
            nn.Linear(embedding_dim, embedding_dim), 
            nn.ReLU(), 
            nn.Linear(embedding_dim, 1)
        )

    def forward(self, x):
        _, glimpse = x # 接收 time_glimpse
        if glimpse.dim() == 3:
            glimpse = glimpse.squeeze(1)
        return self.mlp(glimpse)

class Agent(nn.Module):
    def __init__(self, node_count, time_levels, node_dim,uav_dim,embedding_dim=128):
        super().__init__()
        self.node_count = node_count
        self.time_levels = time_levels
        self.device = mydevice
        self.backbone = Backbone(node_dim,uav_dim,embedding_dim=embedding_dim, time_levels=time_levels, device=self.device)
        self.critic = Critic(embedding_dim=embedding_dim)
        self.actor = Actor()      

    def forward(self, x): 
        backbone_output = self.backbone(x)
        logits_tuple = self.actor(backbone_output)
        value = self.critic(backbone_output)
        return logits_tuple, value

# stateWrapper 保持不变，只需确保 get_mask 正确返回 [batch, 1, node+time]
class stateWrapper:
    def __init__(self, states, device):
        self.device = device
        self.states = {}
        needs_batch = False
        if "observations" in states:
            val = states["observations"]
            dim = val.ndim if hasattr(val, 'ndim') else val.dim()
            if dim == 2: needs_batch = True
        
        for k, v in states.items():
            tensor_v = v.clone().detach().to(self.device) if torch.is_tensor(v) else torch.tensor(v, device=self.device)
            if needs_batch: tensor_v = tensor_v.unsqueeze(0)
            self.states[k] = tensor_v
    def get_current_node(self):
        return self.states["last_node_idx"]
    def get_mask(self):
        mask = (1 - self.states["action_mask"]).to(torch.bool)
        return mask.unsqueeze(1)

class CustomMaskableActorCriticPolicy(MaskableActorCriticPolicy): 
    def __init__(self, observation_space, action_space, lr_schedule, node_count, time_levels,node_dim,uav_dim, embedding_dim=128, *args, **kwargs):
        self.node_count = node_count
        self.time_levels = time_levels
        self.embedding_dim = embedding_dim 
        self.features_dim = embedding_dim 
        self.node_dim = node_dim
        self.uav_dim = uav_dim
        if 'net_arch' not in kwargs: kwargs['net_arch'] = []
        super().__init__(observation_space, action_space, lr_schedule, *args, **kwargs)

    def _build_mlp_extractor(self):
        class DummyMlpExtractor(nn.Module):
            def __init__(self, latent_dim):
                super().__init__()
                self.latent_dim_pi = latent_dim
                self.latent_dim_vf = latent_dim
            def forward(self, x): return x, x
        self.mlp_extractor = DummyMlpExtractor(self.features_dim)
    
    def _build(self, lr_schedule):
        self.agent = Agent(self.node_count, self.time_levels, self.node_dim,self.uav_dim,self.embedding_dim).to(self.device)
        self.action_dist = MaskableMultiCategoricalDistribution([self.node_count, self.time_levels])
        self.action_net = self.agent.actor
        self.value_net = self.agent.critic
        self.optimizer = self.optimizer_class(self.agent.parameters(), lr=lr_schedule(1), **self.optimizer_kwargs)
    def extract_features(self, obs, features_extractor):
        return obs
    def get_distribution(self, obs, action_masks=None):
        logits_tuple, _ = self.agent(obs)
        combined_logits = torch.cat(logits_tuple, dim=-1)
        distribution = self.action_dist.proba_distribution(combined_logits)
        if action_masks is not None:
            distribution.apply_masking(action_masks)
        return distribution
    def forward(self, obs, deterministic=False, action_masks=None):
        logits_tuple, value = self.agent(obs)
        combined_logits = torch.cat(logits_tuple, dim=-1)
        distribution = self.action_dist.proba_distribution(combined_logits)
        if action_masks is not None:
            distribution.apply_masking(action_masks)
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        return actions, value, log_prob
    
    def evaluate_actions(self, obs, actions, action_masks=None):
        logits_tuple, value = self.agent(obs)
        combined_logits = torch.cat(logits_tuple, dim=-1)
        distribution = self.action_dist.proba_distribution(combined_logits)
        if action_masks is not None:
            distribution.apply_masking(action_masks)
        log_prob = distribution.log_prob(actions)
        entropy = distribution.entropy()
        return value.flatten(), log_prob, entropy

    def predict_values(self, obs):
        _, value = self.agent(obs)
        return value.flatten()
    
    def predict(self, obs, state=None, episode_start=None, deterministic=False, action_masks=None):
        """
        重写 predict。SB3 在 predict 时传入的 obs 可能是字典。
        """
        self.set_training_mode(False)
        
        # 转换 obs 为 tensor (如果是 dict)
        with torch.no_grad():
            # 调用重写的 get_distribution
            distribution = self.get_distribution(obs, action_masks=action_masks)
            
            if deterministic:
                actions = distribution.mode()
            else:
                actions = distribution.sample()
        
        # 转回 numpy 以兼容 SB3 接口
        return actions.cpu().numpy(), state