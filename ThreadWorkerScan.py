import zmq
import queue
import time
import random
from SolutionData_pb2 import SolutionData
import logging
import numpy as np
import time
from threading import Thread, Lock
from datetime import datetime
from utils import microseconds_since_unix_epoch_to_datetime
from utils import current_timestamp, \
datetime_to_microseconds_since_unix_epoch

class SolutionSuscriberThread(Thread):    
    __LOG__ = "SolutionSuscriber"
    
    def __init__(self, 
                 host,
                 port,
                 queue,
                 mutex_cola):
        """
            method: can be "mock", "file" or "board"        
        """
        
        self.logger=logging.getLogger(self.__LOG__)

        super().__init__()
        self._run_flag = True  
        self.host = host
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.socket.connect("tcp://{}:{}".format(self.host,self.port))
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self.last_solution_data = None
        self.solution_data_mutex = mutex_cola 
        ################################################ Preguntar donde esta el centro de coordenadas.
        self.queue = queue ### estos es para probar.
        #################################################

    def run(self):
        while self._run_flag:
            socks = dict(self.poller.poll())
            if self.socket in socks and socks[self.socket] == zmq.POLLIN:
                serialized_solution = self.socket.recv(flags=0)                
                self.solution_data_mutex.acquire()
                self.last_solution_data = SolutionData()  
                self.last_solution_data.ParseFromString(serialized_solution)
                if (not self.last_solution_data.solution_array):
                    theta = np.nan
                else:
                    theta = self.last_solution_data.solution_array[1]
                self.queue.put(theta)  # Put the message into the queue
                print(theta)
                self.solution_data_mutex.release()
        self.socket.close()

    def lock_solution(self):
        self.solution_data_mutex.acquire()

    def unlock_solution(self):
        self.solution_data_mutex.release()

    def get_last_solution(self):
        solution_expiration_time_in_seconds = 3
        if self.last_solution_data:
            if (datetime.now()-microseconds_since_unix_epoch_to_datetime(self.last_solution_data.published_timestamp)).seconds > solution_expiration_time_in_seconds:
                self.last_solution_data = None
        return self.last_solution_data
                
    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False
        self.logger.info("Stopping solution suscriber")

if __name__ == "__main__":        
    solution_client = SolutionSuscriberThread( "localhost", 5000 )
    solution_client.start()
    input("Solution suscriber running. Press any key to stop")
    solution_client.stop()
  # Put the message into the queue

class SolutionPublisherThread(Thread):    
    __LOG__ = "SolutionPublisher"
    
    def __init__(self, port):        
        super().__init__()
        
        self.logger=logging.getLogger(self.__LOG__)

        self._run_flag = True  
       
        self.solutions_obtained = 0

        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:{}".format(self.port))
    
    def run(self):
        while self._run_flag: 
            time.sleep(1)

            # Build solution data
            solution_data = SolutionData()
            solution_data.cycle_number =  self.solutions_obtained

            # Completamos con valores dummy
            #solution_data.N_samples_mean = 0
            x_prueba=random.randint(0,72.)
            y_prueba=random.randint(0,49.)
            solution_array = np.array(
                [
                    # intensity, theta, phi, sigma_phi, sigma_theta
                    [ 10., y_prueba, x_prueba , 0.5, 0.6 ],
                    [ 30., 40., 50, 0.8, 0.9 ]
                ]
            )

            # Number of rows and columns of the solution array. 
            solution_data.n_rows = solution_array.shape[0]
            solution_data.n_cols = solution_array.shape[1]

             # Array of floats of n_rows x n_cols serialized to bytes. 
            for i in range(solution_data.n_rows):
                for j in range(solution_data.n_cols):
                    if j == 1:
                        #theta_corregido = (solution_array[i][j] - 4.0) * (-1) 
                        solution_data.solution_array.append(solution_array[i][j])
                    elif j == 2:
                        #rint('phi supuesto:', solution_array[i][j])
                        #phi_corregido = solution_array[i][j] - 1.5
                        #print('phi_corregido:', phi_corregido)
                        solution_data.solution_array.append(solution_array[i][j])
                    else:
                        solution_data.solution_array.append(solution_array[i][j])

            # Timestamps rounded to milliseconds                
            solution_data.elapsed_time_intensities = int(0*1000)
            solution_data.elapsed_time_solution = int(0*1000)

            # Timestamps in microseconds since UNIX epoch.
            solution_data.published_timestamp = datetime_to_microseconds_since_unix_epoch(current_timestamp()) 

            print("Solution published")
            #print('solution: ', solution_array)
            self.socket.send(solution_data.SerializeToString(), flags=0)
            self.solutions_obtained+=1

            
    def stop(self):
        """Sets run flag to False and waits for thread to finish"""
        self._run_flag = False        
        self.logger.info("Stopping solution acquisition")

    def get_cycle_number(self):
        return self.solutions_obtained

    def reset_cycle_number(self):
        self.solutions_obtained = 0

