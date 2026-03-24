import torch


def sigreg_loss(
    Z,                      # (B, D)
    num_projections=1024,   # M (paper default)
    num_t_points=64,        # K (quadrature resolution)
    t_range=(0.2, 4.0),     # paper default
    sigma=1.0,              # weighting bandwidth
    eps=1e-8,
):
    """
    SIGReg for a batch of embeddings Z ∈ R^{B×D}.
    """

    B, D = Z.shape

    # ---- projection matrix U ∈ R^{D × M} ----
    U = torch.randn(D, num_projections, device=Z.device)
    U = U / (U.norm(dim=0, keepdim=True) + eps)

    # ---- projected samples H = ZU ----
    # H ∈ R^{B × M}
    H = Z @ U

    # ---- quadrature grid ----
    t_min, t_max = t_range
    t = torch.linspace(t_min, t_max, num_t_points, device=Z.device)
    delta_t = (t_max - t_min) / num_t_points

    # ---- empirical characteristic function ----
    # shape: (K, B, M)
    exp_term = torch.exp(1j * t[:, None, None] * H[None, :, :])

    # φ_N(t) ∈ (K, M)
    phi_N = exp_term.mean(dim=1)

    # ---- target Gaussian CF ----
    phi_0 = torch.exp(-0.5 * t**2)[:, None]

    # ---- weighting ----
    w = torch.exp(-t**2 / (2 * sigma**2))[:, None]

    # ---- Epps–Pulley statistic ----
    diff = torch.abs(phi_N - phi_0) ** 2
    T_m = (w * diff).sum(dim=0) * delta_t

    # ---- final loss ----
    return T_m.mean()


if __name__ == "__main__":
    z = torch.randn(32, 512)  # Example batch of embeddings 
    loss = sigreg_loss(z)
    print("SIGReg Loss:", loss.item())