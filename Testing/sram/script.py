with open("test-in.txt") as f:
  lines = f.read().splitlines()


cache_tag = []
cache_data = []
for i in range(16):
	cache_tag.append('xxxx')
	cache_data.append('xxxxxxxx')
  
print cache_tag

nLines = []
for i in range(len(lines)):
	line = lines[i][::-1]
	#line = lines[i]
	print line
	we = line[0]  
	wl = line[1:17]

	addr = wl.index('1')

	data_in = line[17:25]
	tag_in = line[25:29]

	if (we=='1'):
		cache_tag[addr] = tag_in
		cache_data[addr] = data_in

	tag_out = cache_tag[addr]
	data_out = cache_data[addr]

	nLine = line + data_out + tag_out
	nLines.append(nLine[::-1])

f = open('test-out.txt', 'w')
for i in nLines:
	f.write('%s\n' % (i))
f.close()
print nLines

