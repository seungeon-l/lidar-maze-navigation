#!/usr/bin/env python3

from time import sleep

import rclpy
from rclpy.node import Node

from create_fundamentals.srv import StoreSong, PlaySong


class ExampleSong(Node):
	def __init__(self):
		super().__init__('example_song')

		self.get_logger().info('Waiting for create driver ...')
		self.store_song_client = self.create_client(StoreSong, 'store_song')
		self.play_song_client = self.create_client(PlaySong, 'play_song')

		while not self.store_song_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for store_song service...')
		while not self.play_song_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for play_song service...')

		self.get_logger().info('Create driver is ready!')

	def call_store_song(self, number, song):
		req = StoreSong.Request()
		req.number = int(number)
		req.song = list(song)
		future = self.store_song_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		return future.result()

	def call_play_song(self, number):
		req = PlaySong.Request()
		req.number = int(number)
		future = self.play_song_client.call_async(req)
		rclpy.spin_until_future_complete(self, future)
		return future.result()


def main(args=None):
	rclpy.init(args=args)
	node = ExampleSong()

	# define silence
	r = 30

	# map note names in the lilypad notation to irobot commands
	c4 = 60
	cis4 = des4 = 61
	d4 = 62
	dis4 = ees4 = 63
	e4 = 64
	f4 = 65
	fis4 = ges4 = 66
	g4 = 67
	gis4 = aes4 = 68
	a4 = 69
	ais4 = bes4 = 70
	b4 = 71
	c5 = 72
	cis5 = des5 = 73
	d5 = 74
	dis5 = ees5 = 75
	e5 = 76
	f5 = 77
	fis5 = ges5 = 78
	g5 = 79
	gis5 = aes5 = 80
	a5 = 81
	ais5 = bes5 = 82
	b5 = 83
	c6 = 84
	cis6 = des6 = 85
	d6 = 86
	dis6 = ees6 = 87
	e6 = 88
	f6 = 89
	fis6 = ges6 = 90

	# define some note lengths
	# change the top MEASURE (4/4 time) to get faster/slower speeds
	MEASURE = 160
	HALF = int(MEASURE / 2)
	Q = int(MEASURE / 4)
	E = int(MEASURE / 8)
	Ed = int(MEASURE * 3 / 16)
	S = int(MEASURE / 16)

	MEASURE_TIME = MEASURE / 64.0

	node.get_logger().info("send songs...")
	# first upload the songs to the irobot...
	node.call_store_song(0, [r, Q])

	node.call_store_song(
		1,
		[
			a4,
			Q,
			a4,
			Q,
			a4,
			Q,
			f4,
			Ed,
			c5,
			S,
			a4,
			Q,
			f4,
			Ed,
			c5,
			S,
			a4,
			HALF,
		],
	)
	node.call_store_song(
		2,
		[
			e5,
			Q,
			e5,
			Q,
			e5,
			Q,
			f5,
			Ed,
			c5,
			S,
			aes4,
			Q,
			f4,
			Ed,
			c5,
			S,
			a4,
			HALF,
		],
	)
	node.call_store_song(
		3,
		[
			a5,
			Q,
			a4,
			Ed,
			a4,
			S,
			a5,
			Q,
			aes5,
			E,
			g5,
			E,
			ges5,
			S,
			f5,
			S,
			ges5,
			S,
		],
	)
	node.call_store_song(
		4,
		[
			r,
			E,
			bes4,
			E,
			ees5,
			Q,
			d5,
			E,
			des5,
			E,
			c5,
			S,
			b4,
			S,
			c5,
			E,
		],
	)
	node.call_store_song(
		5,
		[
			r,
			E,
			f4,
			E,
			aes4,
			Q,
			f4,
			Ed,
			aes4,
			S,
			c5,
			Q,
			a4,
			Ed,
			c5,
			S,
			e5,
			HALF,
		],
	)
	node.call_store_song(
		6,
		[
			r,
			E,
			f4,
			E,
			aes4,
			Q,
			f4,
			Ed,
			c5,
			S,
			a4,
			Q,
			f4,
			Ed,
			c5,
			S,
			a4,
			HALF,
		],
	)

	# once all the songs are uploaded, play them at the right times
	# add a little extra time, b/c otherwise cuts off the end
	sleep(1)
	node.get_logger().info("play songs...")
	node.call_play_song(1)
	sleep(MEASURE_TIME * 2.01)

	node.call_play_song(2)
	sleep(MEASURE_TIME * 2.01)

	node.call_play_song(3)
	sleep(MEASURE_TIME * 1.26)

	node.call_play_song(4)
	sleep(MEASURE_TIME * 1.01)

	node.call_play_song(5)
	sleep(MEASURE_TIME * 1.76)

	node.call_play_song(3)
	sleep(MEASURE_TIME * 1.26)

	node.call_play_song(4)
	sleep(MEASURE_TIME * 1.01)

	node.call_play_song(6)
	sleep(MEASURE_TIME * 1.76)

	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()