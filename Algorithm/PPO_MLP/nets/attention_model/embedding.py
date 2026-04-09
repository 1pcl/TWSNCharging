"""
Problem specific node embedding for static feature.
"""

import torch
import torch.nn as nn


def AutoEmbedding( config):
    """
    Automatically select the corresponding module according to ``problem_name``
    """
    embeddingClass = UAVEmbedding
    embedding = embeddingClass(**config)
    return embedding


class UAVEmbedding(nn.Module):

    def __init__(self, embedding_dim,node_dim,uav_dim):
        super(UAVEmbedding, self).__init__()
        self.context_dim = embedding_dim + uav_dim # Embedding of last node + remaining prize to collect + complting

        self.init_embed = nn.Linear(node_dim, embedding_dim)

    def forward(self, input):
        out = self.init_embed(input)
        return out
