
from collections import deque

import threading
import psutil

class MemoryBoundedDeque:
    def __init__(self, max_fraction=0.2, trim_fraction=0.05):
        self._data = deque() #unbounded deque, length determined by memory usage
        self._max_fraction = max_fraction
        self._trim_fraction = trim_fraction
        self._proc = psutil.Process()
        # Internal locks to ensure value retrieval with values() and append() operations dont clash
        self._lock = threading.Lock()
    
    # return snapshot of deque values as a list
    def values(self):
        with self._lock:
            return list(self._data)

    def append(self, item):
        with self._lock:
            self._data.append(item)
            self._maybe_trim()

    def set_configs(self, max_fraction=None, trim_fraction=None):
        if max_fraction != None:
            self._max_fraction = max_fraction
        if trim_fraction != None:
            self._trim_fraction = trim_fraction

    def front(self):
        if len(self._data) == 0:
            return None
        return self._data[0]

    def back(self):
        if len(self._data) == 0:
            return None
        return self._data[-1]
    
    def should_trim(self):
        # return TRUE if pruning should be done to keep within memory
        # # compare the total allocated memory against the available memory.
        vm = psutil.virtual_memory()
        rss = self._proc.memory_info().rss 
        limit = vm.available * self._max_fraction

        return rss > limit
    
    def trim(self, trim_amount: int=None):
        if len(self._data) == 0:
            return

        # default to making use of the trim fraction to trim a proportional chunk
        if trim_amount == None:
            trim_amount = int(len(self._data) * self._trim_fraction)

        # pop according to the trim amount.
        trim = max(1, trim_amount)
        for _ in range(trim):
            self._data.popleft()

    def _maybe_trim(self):
        # just check the list is not empty
        if len(self._data) == 0:
            return
        
        ### WARNING: An issue with this implementation is that rss does not reduce even if the deque is popped. 
        ### Hence, trimming any amount MORE THAN ONE will result in the deque SHRINKING ratehr than maintaining size.
        ### should_trim() COULD be improved to take conditions using high-low threshold around available memory
        ###     - this would make it more robust to handle cases where available memory drastically reduces suddenly, causing rss ~= available
        ### But for now, maintaining a list size seems to be the expected and intuitive behavior, and should be sufficient in mitigating ~99% of actual OOM cases
        if self.should_trim():
            self.trim(trim_amount=1)

    def __len__(self):
        return len(self._data)

    def __iter__(self):
        return iter(self._data)