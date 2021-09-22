import os

from flask import Flask

from config import basedir, config

def create_app(config_name):
    # init app:
    app = Flask(
        __name__,
        static_url_path = '/static', static_folder = 'static'
    )

    # apply configs:
    app.config.from_object(config[config_name])    
    config[config_name].init_app(app)

    #  logging
    #  ----------------------------------------------------------------
    import logging   
    from .utils.log import LoggingConfiguration
    LoggingConfiguration.set(
        logging.DEBUG if os.getenv('DEBUG') else logging.INFO,
        'app.log', name='WebPortal'
    )

    #  views
    #  ----------------------------------------------------------------    
    from .main import bp as blueprint_main
    app.register_blueprint(blueprint_main)

    return app