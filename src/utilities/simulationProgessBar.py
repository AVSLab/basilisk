from colorama import Fore
from tqdm import tqdm


class SimulationProgressBar:
    """
    Class to generate a BSK progress bar in the terminal window
    """
    def __init__(self, max_value, enable=False):
        self.max_value = max_value
        self.last_update = 0
        self.enable = enable
        self.p = self.pbar()

    def pbar(self):
        return tqdm(
            total=self.max_value,
            desc='Progress: ',
            disable=not self.enable,
            bar_format="%s{l_bar}{bar}|%s" % (Fore.YELLOW, Fore.RESET))

    def update(self, update_value):
        if update_value < self.max_value:
            self.p.update(update_value-self.last_update)
            self.last_update = update_value
        else:
            self.p.update(self.max_value - self.last_update)
            self.last_update = self.max_value

    def markComplete(self):
        if self.update == self.max_value:
            return
        self.p.update(self.max_value-self.last_update)

    def close(self):
        self.p.close()

