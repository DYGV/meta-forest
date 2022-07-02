try:
    import importlib.metadata

    __version__ = importlib.metadata.version("meta-forest")
except ModuleNotFoundError:
    import pkg_resources

    __version__ = pkg_resources.get_distribution("meta-forest").version
