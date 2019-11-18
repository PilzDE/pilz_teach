import curses


class TerminalTextWindow(object):
    _screen = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        key_code = self._screen.getch()
        return key_code if key_code != -1 else None

    def start_page(self):
        self._screen.clear()

    def write_line(self, line_no, message):
        if line_no < 0 or line_no >= self._num_lines:
            raise ValueError('line_no out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * line_no
        x = 10
        for i, text in enumerate(message.split('\n')):
            text = text.ljust(width)
            self._screen.addstr(y + i, x, text)

    def end_page(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()
