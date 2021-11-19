import pandas as pd
import numpy as np
import torch
import torchvision
import os
from skimage import io, transform
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms, utils
import torch.nn as nn
import torch.nn.functional as F


class FaceLandmarksDataset(Dataset):
    """Face Landmarks dataset."""

    def __init__(self, csv_file, root_dir, transform=transforms.Compose([transforms.ToTensor()])):
 #,transforms.Normalize((0.5), (0.5))
        """
        Args:
            csv_file (string): Path to the csv file with annotations.
            root_dir (string): Directory with all the images.
            transform (callable, optional): Optional transform to be applied
                on a sample.
        """
        self.landmarks_frame = pd.read_csv(csv_file)
        self.root_dir = root_dir
        self.transform = transform

    def __len__(self):
        return len(self.landmarks_frame)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        img_name = os.path.join(self.root_dir,
                               'im'+str(idx)+'.jpeg')
        image = io.imread(img_name)
        #image = np.asarray(image,dtype = np.double)

        landmarks = self.landmarks_frame.iloc[idx, 1:]
        landmarks = np.array([landmarks])
        landmarks = landmarks.astype('float').reshape(-1, 1)
        sample = {'image': image, 'landmarks': landmarks}

        if self.transform:
            image = torch.from_numpy(image).double()
            image = torch.reshape(image,(3,200,200)).double()
#             image = torch.reshape(image,(1,))
            landmarks = self.transform(landmarks)
            landmarks = torch.reshape(landmarks,(1,)).double()
            sample = {'image': image, 'landmarks': landmarks}

        return sample

dataset = FaceLandmarksDataset(csv_file='imageLabelWorld.csv', root_dir='imagesWorld')

dataset_test = FaceLandmarksDataset(csv_file='imageLabelWorld.csv', root_dir='imagesWorld    ')
# for i in range(len(transformed_dataset)):
#     sample = transformed_dataset[i]

#     print(i, sample['image'].size(), sample['landmarks'])
#     if i == 3:
#         break
indices = torch.randperm(len(dataset)).tolist()
dataset = torch.utils.data.Subset(dataset, indices[:-150])
print(len(dataset))
dataset_test = torch.utils.data.Subset(dataset_test, indices[-150:])


batch_size = 4
#dataset = torch.utils.data.DataLoader(dataset,batch_size=4, shuffle=True)



# torchvision.datasets.CIFAR10(data, train=True,
                                       # download=True, transform=transform)
# train_size = 0.8*len(dataloader)
# test_size = 0.2*len(dataloader)
# dataloader = DataLoader(transformed_dataset, batch_size=4,
#                         shuffle=True, num_workers=0)

#trainset,testset = torch.utils.data.random_split(dataloader,[train_size,test_size],torch.Generator(device='cuda'))

trainloader = torch.utils.data.DataLoader(dataset, batch_size=batch_size,shuffle=True, num_workers=2)
print(type(trainloader))

#testset = torch.utils.data.DataLoader(testset,train = False,batch_size = batch_size,shuffle=True, num_workers=2 )
# torchvision.datasets.CIFAR10(root='./data', train=False,
#                                        download=True, transform=transform)
testloader = torch.utils.data.DataLoader(dataset_test, batch_size=batch_size,
                                         shuffle=False, num_workers=2)

classes = (0,1,2,3,4)

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.conv1 = nn.Conv2d(3, 6, 5)
        self.pool = nn.MaxPool2d(2, 2)
        self.conv2 = nn.Conv2d(6, 16, 5)
        self.fc1 = nn.Linear(35344, 17000)
        self.fc2 = nn.Linear(17000,7000)
        self.fc3 = nn.Linear(7000,1000)
        self.fc4 = nn.Linear(1000, 120)
        self.fc5 = nn.Linear(120, 5)

    def forward(self, x):
        x = self.pool(F.relu(self.conv1(x)))
        x = self.pool(F.relu(self.conv2(x)))
        x = torch.flatten(x, 1) # flatten all dimensions except batch
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = self.fc5(x)
        return x


net = Net()#.cuda()

import torch.optim as optim

criterion = nn.CrossEntropyLoss()
optimizer = optim.SGD(net.parameters(), lr=0.002, momentum=0.5)

lossList = []
for epoch in range(2):  # loop over the dataset multiple times

    running_loss = 0.0
    for i, data in enumerate(trainloader, 0):
        # get the inputs; data is a list of [inputs, labels]
        inputs, labels= data['image'].double(),data['landmarks']
        
        labels = labels.squeeze(1).type(torch.LongTensor)
        inputs = inputs.double()#.cuda()
        print(type(inputs))
        # zero the parameter gradients
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = net(inputs.float())
        loss = criterion(outputs, labels)
        lossList.append(loss.item())
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if i % 4 == 1:    # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' %
                  (epoch + 1, i + 1, running_loss / 4))
            running_loss = 0.0

print('Finished Training')

import matplotlib.pyplot as plt
plt.plot(lossList)
plt.xlabel('input number')
plt.ylabel('training loss')
plt.show()


PATH = './cifar_net.pth'
torch.save(net.state_dict(), PATH)

# for i, data in enumerate(testloader):
#     # get the inputs; data is a list of [inputs, labels]
#     images, labels= data['image'].double(),data['landmarks']
#     #print('GroundTruth: ', ' '.join('%5s' % labels[i]))
#     labels = labels.squeeze(1).type(torch.LongTensor)

lossList = []

net.load_state_dict(torch.load(PATH))
#outputs = net(images.float())


plt.plot(lossList)
plt.xlabel('input number')
plt.ylabel('training loss')
plt.show()

_, predicted = torch.max(outputs, 1)


correct = 0
total = 0
# since we're not training, we don't need to calculate the gradients for our outputs
with torch.no_grad():
    for data in testloader:
        images, labels = data['image'],data['landmarks']
        labels = labels.squeeze(1).type(torch.LongTensor)
        # calculate outputs by running images through the network
        outputs = net(images.float())
        # the class with the highest energy is what we choose as prediction
        _, predicted = torch.max(outputs.data, 1)
        total += labels.size(0)
        correct += (predicted == labels).sum().item()

print('Accuracy of the network on the  test images: %d %%' % (
    100 * correct / total))

print(correct)

# prepare to count predictions for each class
correct_pred = {classname: 0 for classname in classes}
total_pred = {classname: 0 for classname in classes}

# again no gradients needed
with torch.no_grad():
    for data in testloader:
        images, labels = data['image'],data['landmarks']
        labels = labels.squeeze(1).type(torch.LongTensor)
        outputs = net(images.float())
        _, predictions = torch.max(outputs, 1)
        # collect the correct predictions for each class
        for label, prediction in zip(labels, predictions):
            if label == prediction:
                correct_pred[classes[label]] += 1
            total_pred[classes[label]] += 1


# print accuracy for each class
for classname, correct_count in correct_pred.items():
    accuracy = 100 * float(correct_count) / total_pred[classname]
    print("Accuracy for class {:5s} is: {:.1f} %".format(str(classname),
                                                   accuracy))

validationset = FaceLandmarksDataset(csv_file = 'land.csv',root_dir='imagesWorld')

outs = []
with torch.no_grad():
    for data in testloader:
        images, labels = data['image'],data['landmarks']
        labels = labels.squeeze(1).type(torch.LongTensor)
        # calculate outputs by running images through the network
        outputs = net(images.float())
        _, predicted = torch.max(outputs.data, 1)
        outs.append(predicted)
print(outs)