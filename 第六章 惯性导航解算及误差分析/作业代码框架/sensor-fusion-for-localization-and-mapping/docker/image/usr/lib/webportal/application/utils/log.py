import sys
import logging
import logging.handlers


#These are the sequences need to get colored ouput
RESET_SEQ = "\033[0m"
COLOR_SEQ = "\033[1;%dm"
BOLD_SEQ = "\033[1m"


class ColoredFormatter(logging.Formatter):
    # the terminal has 8 colors with codes from 0 to 7
    BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)

    # the background is set with 40 plus the number of the color,
    # and the foreground with 30
    COLORS = {
        'WARNING':  COLOR_SEQ % (30 + YELLOW) + 'WARN' + RESET_SEQ,
        'INFO':     COLOR_SEQ % (30 + WHITE) + 'INFO' + RESET_SEQ,
        'DEBUG':    COLOR_SEQ % (30 + BLUE) + 'DEBUG' + RESET_SEQ,
        'CRITICAL': COLOR_SEQ % (30 + YELLOW) + 'CRITI' + RESET_SEQ,
        'ERROR':    COLOR_SEQ % (30 + RED) + 'ERROR' + RESET_SEQ,
    }

    def __init__(self, msg, use_color=True):
        logging.Formatter.__init__(self, msg)
        self.use_color = use_color

    def format(self, record):
        if self.use_color:
            # defaults to plain text with no color:
            record.levelname = self.COLORS.get(record.levelname, record.levelname)
        return logging.Formatter.format(self, record)


class LoggingConfiguration(object):
    COLOR_FORMAT = (
        "[%(asctime)s][%(threadName)-9s][%(levelname)s] "
        "%(message)s (" + BOLD_SEQ + "%(filename)s" + RESET_SEQ + ":%(lineno)d)"
    )
    NO_COLOR_FORMAT = (
        "[%(asctime)s][%(threadName)-9s][%(levelname)s] "
        "%(message)s " + "(%(filename)s:%(lineno)d)"
    )
    FILE_FORMAT = (
        "[%(asctime)s][%(threadName)-9s][%(levelname)s] "
        "%(message)s "
    )

    @classmethod
    def set(cls, log_level, log_filename, append=None, **kwargs):
        """ configure a rotating file logging
        """
        logger = logging.getLogger()
        logger.setLevel(log_level)

        COLOR_FORMAT = cls.COLOR_FORMAT
        NO_COLOR_FORMAT = cls.NO_COLOR_FORMAT
        FILE_FORMAT = cls.FILE_FORMAT
        if 'name' in kwargs:
            COLOR_FORMAT = COLOR_FORMAT.replace(
                '%(threadName)-9s', '%-9s' % (kwargs['name'])
            )
            NO_COLOR_FORMAT = NO_COLOR_FORMAT.replace(
                '%(threadName)-9s', '%-9s' % (kwargs['name'])
            )
            FILE_FORMAT = FILE_FORMAT.replace(
                '%(threadName)-9s', '%s' % (kwargs['name'])
            )

        # log to rotating file
        try:
            log_handler = logging.handlers.RotatingFileHandler(
                log_filename,
                mode='a+',
                backupCount=3
            )
            log_handler = logging.FileHandler(
                log_filename,
                mode='a+'
            )
            log_handler.setFormatter(ColoredFormatter(FILE_FORMAT, False))
            log_handler.setLevel(log_level)
            logger.addHandler(log_handler)
            if not append:
                # create a new log file:
                log_handler.doRollover()
        except:
            pass

        # log to sys.stderr using log level passed through command line
        if log_level != logging.NOTSET:
            log_handler = logging.StreamHandler(sys.stdout)
            if sys.platform.find('linux') >= 0:
                formatter = ColoredFormatter(COLOR_FORMAT)
            else:
                formatter = ColoredFormatter(NO_COLOR_FORMAT, False)
            log_handler.setFormatter(formatter)
            log_handler.setLevel(log_level)
            logger.addHandler(log_handler)
