from flask import request, render_template

from flask import current_app
from . import bp

import subprocess
import time


@bp.route('/')
def index():
    """ Get client view portal size for noVNC
    """
    current_app.logger.info('Get client view portal size.')

    return render_template('index.html')


@bp.route('/redirect.html')
def redirect():
    """ Render noVNC portal
    """
    if current_app.config['VIEW_PORTAL_INITIALIZED']:
        current_app.logger.info('Use existing view portal configuration.')
        return render_template('redirect.html')

    # parse view portal size:
    env = {'width': 1024, 'height': 768}
    if 'width' in request.args:
        env['width'] = request.args['width']
    if 'height' in request.args:
        env['height'] = request.args['height']
    
    # change desktop resolution:
    subprocess.check_call(
        r"sed -i 's#^command=/usr/bin/Xvfb.*$#command=/usr/bin/Xvfb :1 -screen 0 {width}x{height}x16#' /etc/supervisor/conf.d/lxde.conf".format(**env),
        shell=True
    )

    # reload desktop:
    subprocess.check_call(r"supervisorctl update", shell=True)

    # make sure all the daemon processes are running:
    for i in range(current_app.config['MAX_TRYOUT']):
        # get the number of running supervisord processes:
        num_running = subprocess.check_output(r"supervisorctl status | grep RUNNING | wc -l", shell=True)
        if num_running.strip() == str(current_app.config['NUM_RUNNING']):
            # mark as initialized:
            current_app.config['VIEW_PORTAL_INITIALIZED'] = True
            current_app.logger.info(
                (
                    'Set view portal size to (%d, %d) '
                    'after tryout %d'
                ),
                int(env['width']), int(env['height']), i + 1
            )
            return render_template('redirect.html')
        # sleep and try again:
        time.sleep(current_app.config['WAIT_TIME'])

    return render_template('redirect.html')

@bp.route('/aboutus.html')
def aboutus():
    """ General info for testing
    """
    current_app.logger.info('Get aboutus general info.')

    return render_template('aboutus.html')