import glob, os

imgs_dir = '/home/ucar/catkin_test_ws/src/darknet-master/total/img'
print(imgs_dir)

percentage_test = 10

file_train = open('/home/ucar/catkin_test_ws/src/darknet-master/total/train.txt', 'w')
file_test = open('/home/ucar/catkin_test_ws/src/darknet-master/total/test.txt', 'w')

counter = 1
index_test = round(100 / percentage_test)
for pathAndFilename in glob.iglob(os.path.join(imgs_dir, "*.jpg")):
    title, ext = os.path.splitext(os.path.basename(pathAndFilename))

    if counter == index_test:
        counter = 1
        file_test.write(imgs_dir + "/" + title + '.jpg' + "\n")
    else:
        file_train.write(imgs_dir + "/" + title + '.jpg' + "\n")
        counter = counter + 1

