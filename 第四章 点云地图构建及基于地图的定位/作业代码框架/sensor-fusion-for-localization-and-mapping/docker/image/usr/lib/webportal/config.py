import os

# root dir of app:
basedir = os.path.abspath(os.path.dirname(__file__))

class Config:
    # security:
    SECRET_KEY = os.environ.get('SECRET_KEY') or os.urandom(32)

    # view portal initialization:
    VIEW_PORTAL_INITIALIZED = False
    # the number of running supervisord processes:
    NUM_RUNNING = 6
    # max num. of tryouts:
    MAX_TRYOUT = 20
    WAIT_TIME = 1

    @staticmethod
    def init_app(app):
        """ integrate with app factory
        """
        pass


class DevelopmentConfig(Config):
    DEBUG = True


class TestingConfig(Config):
    TESTING = True


class ProductionConfig(Config):
    PRODUCTION=True


# configs:
config = {
    'development': DevelopmentConfig,
    'testing': TestingConfig,
    'production': ProductionConfig,

    'default': DevelopmentConfig
}