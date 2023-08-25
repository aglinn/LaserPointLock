import concurrent.futures

import cv2
import multiprocessing
import threading
import numpy as np


class ProcessingMultithread:
    def __init__(self, process_frame_func, num_threads_per_consumer, num_consumer_processes,
                 max_frames_in_queue, batch_size=50):
        """
        Initialize the Processing class with the provided parameters.

        Args:
            capture (cv2.VideoCapture): OpenCV capture object for reading frames.
            process_frame_func (function): Function to process each frame.
            num_threads_per_consumer (int): Number of threads per consumer process.
            num_consumer_processes (int): Number of consumer processes.
            max_frames_in_queue (int): Maximum number of frames allowed to be put on the queue
        """
        self.process_frame_func = process_frame_func
        self.num_threads_per_consumer = num_threads_per_consumer
        self.num_consumer_processes = num_consumer_processes
        self.max_frames_in_queue = max_frames_in_queue
        self.num_frames = None
        self.batch_size = batch_size
        return

    def process_frame(self, frame_queue, num_batches):
        """
        Function to process each frame from the frame queue.

        Args:
            frame_queue (multiprocessing.JoinableQueue): Queue containing frames to process.
            result_queue (multiprocessing.JoinableQueue): Queue to store the processed frames.
        """
        results = []
        i = 0
        while True:
            # Get a frame and its number from the queue
            frames = frame_queue.get()

            # Process the frame using the provided function
            with concurrent.futures.ThreadPoolExecutor() as executor:
                for result in executor.map(self.process_frame_func, frames):
                    results.append(result)

            # Mark the task as done for the frame queue
            frame_queue.task_done()
            i += 1
            if i == num_batches:
                break
        executor.shutdown()
        return results

    def run_processing(self, capture):
        """
        Run the frame processing calculation using multiple threads and processes.

        Returns:
            list: List of processed frames in the same order as captured.
        """
        self.num_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

        # Create a shared queue to hold the frames
        frame_queue = multiprocessing.JoinableQueue(int(np.floor(self.max_frames_in_queue/self.batch_size)))

        # Create the producer thread cannot pickle capture object so cannot move that to a new process
        producer_thread = threading.Thread(target=self.producer, args=(capture, frame_queue, self.batch_size))
        producer_thread.start()
        print("Making consumers now.")
        # Create and start the consumers on their threads
        results = self.process_frame(frame_queue, int(np.ceil(self.num_frames/self.batch_size)))

        # Wait for the producer process to finish
        producer_thread.join()

        # Wait for all the consumer threads to finish
        frame_queue.join()
        return results

    @staticmethod
    def producer(capture, frame_queue, batch_size):
        """
        Producer function to read frames from the capture object and put them in the frame queue.

        Args:
            frame_queue (multiprocessing.JoinableQueue): Queue to store the frames.
        """
        frames = []
        num_frames = 0
        while True:
            # Read a frame from the capture object
            ret, frame = capture.read()
            if not ret:
                if frames:
                    frame_queue.put(frames)
                break
            frames.append(frame)
            num_frames += 1
            if num_frames == batch_size:
                # Put the frames into the queue
                frame_queue.put(frames)
                frames = []
                num_frames = 0
        return


class Processing:
    def __init__(self, process_frame_func, num_threads_per_consumer, num_consumer_processes,
                 max_frames_in_queue):
        """
        Initialize the Processing class with the provided parameters.

        Args:
            capture (cv2.VideoCapture): OpenCV capture object for reading frames.
            process_frame_func (function): Function to process each frame.
            num_threads_per_consumer (int): Number of threads per consumer process.
            num_consumer_processes (int): Number of consumer processes.
            max_frames_in_queue (int): Maximum number of frames allowed to be put on the queue
        """
        self.process_frame_func = process_frame_func
        self.num_threads_per_consumer = num_threads_per_consumer
        self.num_consumer_processes = num_consumer_processes
        self.max_frames_in_queue = max_frames_in_queue
        self.num_frames = None
        return

    def process_frame(self, frame_queue, result_queue):
        """
        Function to process each frame from the frame queue.

        Args:
            frame_queue (multiprocessing.JoinableQueue): Queue containing frames to process.
            result_queue (multiprocessing.JoinableQueue): Queue to store the processed frames.
        """
        while True:
            # Get a frame and its number from the queue
            data = frame_queue.get()

            if data is None:
                # No more frames to process, exit the thread
                frame_queue.task_done()
                break

            frame_number, frame = data

            # Process the frame using the provided function
            processed_frame = self.process_frame_func(frame)

            # Put the processed frame and its number into the result queue
            result_queue.put((frame_number, processed_frame))

            # Mark the task as done for the frame queue
            frame_queue.task_done()
        return

    def run_processing(self, capture):
        """
        Run the frame processing calculation using multiple threads and processes.

        Returns:
            list: List of processed frames in the same order as captured.
        """
        self.num_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))

        # Create a shared queue to hold the frames
        frame_queue = multiprocessing.JoinableQueue(self.max_frames_in_queue)

        # Create a shared queue to hold the results (processed frames)
        result_queue = multiprocessing.JoinableQueue()

        # Create the producer thread cannot pickle capture object so cannot move that to a new process
        #producer_process = multiprocessing.Process(target=self.producer, args=(frame_queue,))
        producer_thread = threading.Thread(target=self.producer, args=(capture, frame_queue))
        producer_thread.start()
        print("Making consumers now.")
        # Create and start the consumers on their processes with their threads
        if self.num_consumer_processes <= 0:
            self.consumer(frame_queue, result_queue)
        else:
            consumer_processes = []
            for _ in range(self.num_consumer_processes):
                process = multiprocessing.Process(target=self.consumer, args=(frame_queue, result_queue))
                # threading.Thread(target=self.consumer, args=(frame_queue, result_queue))
                process.start()
                consumer_processes.append(process)

        # Wait for the producer process to finish
        producer_thread.join()

        # Wait for all the consumer threads to finish
        frame_queue.join()
        # Collect the processed frames from the result queue and sort them based on the frame numbers
        results = self.unpack_results(result_queue)
        result_queue.join()
        return results

    @staticmethod
    def producer(capture, frame_queue):
        """
        Producer function to read frames from the capture object and put them in the frame queue.

        Args:
            frame_queue (multiprocessing.JoinableQueue): Queue to store the frames.
        """
        frame_number = 0
        while True:
            # Read a frame from the capture object
            ret, frame = capture.read()

            if not ret:
                break
            # Put the frame and its number into the queue
            frame_queue.put((frame_number, frame))
            frame_number += 1
        return

    def consumer(self, frame_queue, result_queue):
        """
        Consumer function to process frames from the frame queue and store the results in the result queue.

        Args:
            frame_queue (multiprocessing.JoinableQueue): Queue containing frames to process.
            result_queue (multiprocessing.JoinableQueue): Queue to store the processed frames.
        """
        consumer_threads = []
        for _ in range(self.num_threads_per_consumer):
            thread = threading.Thread(target=self.process_frame, args=(frame_queue, result_queue))
            thread.start()
            consumer_threads.append(thread)
        return

    def unpack_results(self, results_queue):
        results = [None]*self.num_frames
        i = 0
        while True:
            data = results_queue.get()
            results[data[0]] = data[1]
            results_queue.task_done()
            i += 1
            if i == self.num_frames:
                break
        return results