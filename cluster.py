import pandas as pd
from pyclustering.cluster import cluster_visualizer, cluster_visualizer_multidim
from pyclustering.cluster.xmeans import xmeans
from pyclustering.cluster.center_initializer import kmeans_plusplus_initializer
import numpy as np
import matplotlib.pyplot as plt

print('converting cuboid')
dataFrame1 = pd.read_csv('CuboidWorld1.csv')#,converters={'image': eval})
# df1 = dataFrame1.groupby('point')
# print(df1.head())
# df1 = pd.DataFrame(df1)
# df1.to_csv('aaaaa.csv')
points = dataFrame1['point']
points = points.tolist()
print(len(points),'table')
del dataFrame1['realDelta']
del dataFrame1['PointCloud']

print('converting wall')
dataFrame2 = pd.read_csv('TableWorld2.csv')#,converters={'image': eval})
points = dataFrame2['point']SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
image = dataFrame2['point']
points = points.tolist()
print(len(points),'wall')
del dataFrame2['realDelta']
del dataFrame2['PointCloud']

print('converting table')
dataFrame3 = pd.read_csv('WallWorld3.csv')#,converters={'image': eval})
image = dataFrame3['point']
points = dataFrame3['point']
points = points.tolist()
print(len(points),'cuboid')
del dataFrame3['realDelta']
del dataFrame3['PointCloud']

dataFrame = dataFrame1.append(dataFrame2)
dataFrame = dataFrame.append(dataFrame3)
points = dataFrame['point']
points = points.tolist()
# image = pd.read_pickle('images.zip')
#image = dataFrame['image']
image.to_pickle('images.zip')
images = image.tolist()
del dataFrame['point']
del dataFrame['image']
sample = dataFrame.values.tolist()
# Prepare initial centers - amount of initial centers defines amount of clusters from which X-Means will
# start analysis.
amount_initial_centers = 3
initial_centers = kmeans_plusplus_initializer(sample, amount_initial_centers).initialize()
# Create instance of X-Means algorithm. The algorithm will start analysis from 2 clusters, the maximum
# number of clusters that can be allocated is 20.
xmeans_instance = xmeans(sample, initial_centers, 5)
xmeans_instance.process()
# Extract clustering results: clusters and their centers
clusters = xmeans_instance.get_clusters()
print(len(clusters))
print(clusters)
centers = xmeans_instance.get_centers()
#print('clusters', clusters)
print('centers length',len(centers))
df = pd.DataFrame(columns=['label'])
for k in range(len(sample)):
    for j in range(len(clusters)):
        for i in range(len(clusters[j])):
            if clusters[j][i] == k:
                newRow = pd.DataFrame(np.array([[j]]),columns = ['label'])
                df = df.append(newRow)
print(df)
#df.to_csv('imageLabelWorld.csv')
# df.to_csv('WallWorld.csv',index=False)
# print('centers',centers)
# Visualize clustering results
# visualizer = cluster_visualizer_multidim()
# visualizer.append_clusters(clusters, sample)
# visualizer.append_cluster(centers, None, marker='*', markersize=10)
# visualizer.show()

pointsLen = []
for j in range(len(clusters)):
    pointsLen.append(len(clusters[j]))
print(pointsLen)
allPoints = []
for i in range(len(clusters)):
    pointsList = []
    for cluster in clusters[i]: 
        p = points[cluster].strip('][').split()
        for j in range(3):
            p[j] = float(p[j])
        pointsList.append(p)
    allPoints.append(pointsList)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
colors = ['red','green','blue','yellow','purple','orange','black','gray','magenta','olive','dodgerblue','lime','indigo','pink','peru','crimson','brown','rosybrown','darkred','navy']
plt.xlabel('x(m)')
plt.ylabel('y(m)')

for i in range(len(clusters)):
    cluster = allPoints[i]
    xs = [point[0] for point in cluster]
    ys = [point[1] for point in cluster]
    zs = [point[2] for point in cluster]
    ax.scatter(xs,ys,zs,c = colors[i])
ax.set_xlabel('X(m)')
ax.set_ylabel('Y(m)')
ax.set_zlabel('Z(m)')

fig = plt.figure()
ax = fig.add_subplot()
for i in range(len(clusters)):
    for j in range(len(clusters[i])):
        ax.scatter(i,clusters[i][j])
plt.xlabel("Cluster Number")
plt.ylabel("Points belonging to the cluster")   
plt.show()




