from threading import RLock
from typing import List

class MovingAverage():
    def __init__(self, max_size_of_buf : int) -> None:
        self.__size_of_buf = max_size_of_buf
        self.__data : List[float] = [0]*self.__size_of_buf
        self.__last_buffer_value = 0
        self.__overwritten_buffer_value = 0
        self.__placing_value = 0
        self.__current_index = 0
        self.__averaging_buffer_offset = 0
        self.__num_valid_samples = 0
        self.__current_average = 0
        self.__lock = RLock()
        pass


    def add_sample(self, sample : float):
        with self.__lock:
            self.__last_buffer_value = self.__data[self.__current_index]
            self.__current_index += 1
            if self.__current_index >= self.__size_of_buf:
                self.__current_index = 0
            
            self.__overwritten_buffer_value = self.__data[self.__current_index]
            self.__placing_value = sample + self.__last_buffer_value
            if self.__current_index == 0:
                self.__averaging_buffer_offset = self.__overwritten_buffer_value
                self.__placing_value -= self.__averaging_buffer_offset

            self.__num_valid_samples += 1
            self.__num_valid_samples = self.__size_of_buf if self.__num_valid_samples > self.__size_of_buf else self.__num_valid_samples
            self.__data[self.__current_index] = self.__placing_value
            self.__current_average = (self.__placing_value - self.__overwritten_buffer_value + self.__averaging_buffer_offset) / self.__num_valid_samples
            return self.__current_average


    def get_average(self) -> float:
        with self.__lock:
            return self.__current_average