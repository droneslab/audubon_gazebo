
import os
import cv2
tags_list = sorted([x for x in os.listdir('tags/')])
# print(len(tags_list))
image_list = sorted(['tags/'+x for x in tags_list])
image_list = image_list[1:]
counter = 1
for i in tags_list:
    with open('april0.dae','r') as in_file, open('daes/april{}.dae'.format(counter),'w+') as out_file:
        for line in in_file:
            if('tag36_11_00000' in line):
                if counter > 100:
                    line = line.replace('tag36_11_00000','tag36_11_00{}'.format(counter))
                elif counter > 10:
                    line = line.replace('tag36_11_00000','tag36_11_000{}'.format(counter))
                else:
                    line = line.replace('tag36_11_00000','tag36_11_000{}'.format(counter))
            out_file.write(line)
    blurred = cv2.imread('tags/'+i)
    big = cv2.resize(blurred,(300,300),interpolation=cv2.INTER_NEAREST)
    cv2.imwrite('hi_res_tags/'+i,big)
    counter += 1
        