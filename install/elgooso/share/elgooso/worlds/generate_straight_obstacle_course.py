#!/usr/bin/env python3
import random

class Box:
	def __init__(self, prefix, px):
		self.prefix = prefix
		self.sx = random.random()
		self.sy = random.random()
		self.sz = 0.5
		self.px = px
		self.py = random.uniform(-1.0 + self.sy/2.0, 1.0 - self.sy/2.0)
		self.pz = 0.25

def main():
	f = open('world_demo.xacro', 'a')
	n_boxes = 10
	for i in range(n_boxes):
		px = 2*i + 1
		obj = Box(i, px)
		f.write(f"		<xacro:box prefix='{obj.prefix}' sx='{obj.sx}' sy='{obj.sy}' sz='{obj.sz}' px='{obj.px}' py='{obj.py}' pz='{obj.pz}'/>\n")
	f.write('	</world>\n')
	f.write('</sdf>')
	f.close()
	
	
if __name__== "__main__":
	main()
