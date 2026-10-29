"""Backend navigasi orbit_adapter (ADR-002). Backend dipilih per robot lewat key `backend`."""


def make_backend(kind, name, config, use_sim_time):
    if kind == 'nav2':
        from .nav2 import Nav2Backend
        return Nav2Backend(name, config, use_sim_time)
    raise ValueError(f'backend tidak dikenal untuk {name}: {kind!r} (tersedia: nav2)')
