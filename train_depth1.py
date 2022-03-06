#1
import os
import cv2
import torch
import torchvision
import torchvision.transforms as transforms

import matplotlib.pyplot as plt
import numpy as np


# 2.
import torch.nn as nn
import torch.nn.functional as F

# 3.
import torch.optim as optim

# Set the split

split = (0.7,0.2,0.1)

# # show images
# imshow(torchvision.utils.make_grid(images))
# # print labels
# print(' '.join(f'{classes[labels[j]]:5s}' for j in range(batch_size)))

# 1. Get the training, testing data set. Define classes

data_directory = "./training_data/data/exp1/episodes/1/1/"
dir_arr = os.listdir(data_directory)

classes = (0, 0.05, 0.175, 0.3, 0.425, 0.55, 0.675, 0.8, 1)

plt.figure()
plt.title("Accuracy graph")
plt.show(block=False)
# 2. Define a Convolutional Neural Network

class Net(nn.Module):
	def __init__(self):
		super().__init__()
		self.conv1 = nn.Conv2d(3, 16, kernel_size=8,stride=4)
		self.pool = nn.MaxPool2d(1, 1)
		self.conv2 = nn.Conv2d(16, 32, kernel_size=4,stride=2)
		in_size = (84-8)/4+1
		in_size = int((in_size-4)/2+1)
		self.fc1 = nn.Linear(32 * in_size* in_size, 256)
		self.fc2 = nn.Linear(256, 128)
		self.out1 = nn.Linear(128, 64)
		self.out2 = nn.Linear(128, 64)
		self.out3 = nn.Linear(128, 64)
		self.out4 = nn.Linear(128, 64)
		self.out5 = nn.Linear(128, 64)
		self.out6 = nn.Linear(128, 64)
		self.out7 = nn.Linear(128, 64)
		self.out8 = nn.Linear(128, 64)


	def forward(self, x):
		x = (F.relu(self.conv1(x)))
		x = (F.relu(self.conv2(x)))
		x = torch.flatten(x, 1) # flatten all dimensions except batch
		x = F.relu(self.fc1(x))
		x = F.relu(self.fc2(x))
		y = []
		y.append((self.out1(x)))
		y.append((self.out2(x)))
		y.append((self.out3(x)))
		y.append((self.out4(x)))
		y.append((self.out5(x)))
		y.append((self.out6(x)))
		y.append((self.out7(x)))
		y.append((self.out8(x)))
		y= torch.swapaxes(torch.stack(y), 0, 1)
		return F.softmax(y,dim=0)


net = Net()

device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')

# 2.5. Assuming that we are on a CUDA machine, this should print a CUDA device:

print(device)

net.to(device)
trans_tensor = torchvision.transforms.ToTensor()

# 3. Define a Loss function and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(net.parameters(), lr=0.001, momentum=0.9)
mini_batch = 4;
accuracy=[];
plot_frequency=1000;
for epoch in range(2):  # loop over the dataset multiple times

	running_loss = 0.0
	file_index = 1
	while(file_index < (len(dir_arr)/2*split[0])):
		# get the inputs; data is a list of [inputs, labels]
		list_inputs = []
		list_targets = []

		for j in range(mini_batch):
			list_inputs.append(trans_tensor(cv2.imread(data_directory + "0-1-Vis-rgb-" + str(file_index + j) + ".png")))
			list_targets.append(torch.sum(torch.stack([((trans_tensor(cv2.imread(data_directory + "0-1-Vis-depth-" + str(file_index + j) + ".png",cv2.IMREAD_GRAYSCALE)).flatten()/255)**10 >= depth_class) for depth_class in classes]),dim=0)-1)

		inputs = torch.stack(list_inputs)
		targets = torch.stack(list_targets)

		inputs, targets = inputs.to(device), targets.to(device)

		file_index=file_index+mini_batch;

		# zero the parameter gradients
		optimizer.zero_grad()
		# forward + backward + optimize
		outputs = net(inputs)
		loss = criterion(outputs, targets)
		loss.backward()
		optimizer.step()

		# print statistics
		running_loss += loss.item()
		# print(int(file_index-1) % 200)
		if(int(file_index-1) % 200 == 0):    # print every 200 mini-batches
			print("Mini_batch : {}".format(file_index//mini_batch))	
		
		if(int(file_index-1) % plot_frequency == 0):
			with torch.no_grad():	
				list_val_inputs = []
				list_val_targets = []
				correct_pred = 0
				total_pred = 0
				test_cel = 0;

				for j in range(1,int(len(dir_arr)/2*split[0])): #range(int(len(dir_arr)/2*(split[0])),int(len(dir_arr)/2*(split[0]+split[1]))):#
					list_val_inputs.append(trans_tensor(cv2.imread(data_directory + "0-1-Vis-rgb-" + str(j) + ".png")))
					list_val_targets.append(torch.sum(torch.stack([((trans_tensor(cv2.imread(data_directory + "0-1-Vis-depth-" + str(j) + ".png",cv2.IMREAD_GRAYSCALE)).flatten()/255)**10 >= depth_class) for depth_class in classes]),dim=0)-1)
				val_inputs = torch.stack(list_val_inputs)
				val_targets = torch.stack(list_val_targets)

				val_inputs, val_targets = val_inputs.to(device), val_targets.to(device)
				# calculate outputs by running images through the network
				val_outputs = net(val_inputs)
				_, predictions = torch.max(val_outputs, 1)
				# the class with the highest energy is what we choose as prediction
				# test_cel += criterion(outputs,targets)
				for target, prediction in zip(val_targets, predictions):
					correct_pred += torch.sum((target==prediction).float())
					total_pred += torch.sum(torch.ones_like(prediction))
				accuracy.append(100 * float(correct_pred) / float(total_pred))
				print(accuracy)
				plt.cla()
				plt.plot(range(epoch*int(len(dir_arr)/2*split[0])//mini_batch*mini_batch//plot_frequency + (file_index-1)//plot_frequency),accuracy)
				plt.pause(0.1)

		if(int(file_index-1) % 2000 == 0):    # print every 2000 mini-batches
			print(f'[{epoch + 1}, {file_index-1 + 1:5d}] loss: {running_loss / 200:.3f}')
			running_loss = 0.0
print('Finished Training')

PATH = './depth_net.pth'
torch.save(net.state_dict(), PATH)


## See https://pytorch.org/docs/stable/notes/serialization.html for more details on saving and loading PyTorch models

# 4. Get CEL and Accuracy with Test set


correct_pred = 0
total_pred = 0
test_cel = 0;
# since we're not training, we don't need to calculate the gradients for our outputs
with torch.no_grad():	
		list_test_inputs = []
		list_targets = []

		for j in range(int(len(dir_arr)/2*(split[0]+split[1])),int(len(dir_arr)/2)):
			list_test_inputs.append(trans_tensor(cv2.imread(data_directory + "0-1-Vis-rgb-" + str(j) + ".png")))
			list_targets.append(torch.sum(torch.stack([((trans_tensor(cv2.imread(data_directory + "0-1-Vis-depth-" + str(j) + ".png",cv2.IMREAD_GRAYSCALE)).flatten()/255)**10 >= depth_class) for depth_class in classes]),dim=0)-1)
		test_inputs = torch.stack(list_test_inputs)
		targets = torch.stack(list_targets)

		test_inputs, targets = test_inputs.to(device), targets.to(device)
		# calculate outputs by running images through the network
		outputs = net(test_inputs)
		_, predictions = torch.max(outputs, 1)
		# the class with the highest energy is what we choose as prediction
		test_cel += criterion(outputs,targets)
		for target, prediction in zip(targets, predictions):
			correct_pred += torch.sum((target==prediction).float())
			total_pred += torch.sum(torch.ones_like(prediction))
		

accuracy = 100 * float(correct_pred) / total_pred
print(f'Accuracy is {accuracy:.1f} %')

print(f'Cross Entropy Loss of the network on the test images: {test_cel}')


# # prepare to count predictions for each class
# correct_pred = {classname: 0 for classname in classes}
# total_pred = {classname: 0 for classname in classes}

# # again no gradients needed
# with torch.no_grad():
# 	for data in testloader:
# 		images, labels = data
# 		outputs = net(images)
# 		_, predictions = torch.max(outputs, 1)
# 		# collect the correct predictions for each class
# 		for label, prediction in zip(labels, predictions):
# 			if label == prediction:
# 				correct_pred[classes[label]] += 1
# 			total_pred[classes[label]] += 1