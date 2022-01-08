
class SingletonKeta(type):
    """
    Makes sure every module uses the same data
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        raise ValueError('NO SINGLETON')
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]
