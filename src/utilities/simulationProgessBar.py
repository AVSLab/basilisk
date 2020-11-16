from tqdm import tqdm
from colorama import Fore

class SimulationProgressBar:
    def __init__(self, max_value, disable=True):
        self.max_value = max_value
        self.last_update = 0
        self.disable = disable
        self.p = self.pbar()

    def pbar(self):
        return tqdm(
            total=self.max_value,
            desc='Progress: ',
            disable=self.disable,
            bar_format="%s{desc}|{bar:50}|{percentage:3.0f}%%%s" % (Fore.YELLOW, Fore.RESET))

    def update(self, update_value):
        self.p.update(update_value-self.last_update)
        self.last_update = update_value

    def close(self):
        self.p.close()

