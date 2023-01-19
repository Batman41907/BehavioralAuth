import numpy as np # linear algebra
import pandas as pd # data processing, CSV file I/O (e.g. pd.read_csv)
import seaborn as sns
import matplotlib.pyplot as plt
# Input data files are available in the "../input/" directory.
# For example, running this (by clicking run or pressing Shift+Enter) will list the files in the input directory

from subprocess import check_output

from sklearn.linear_model import LogisticRegression  # for Logistic Regression algorithm
from sklearn.model_selection import train_test_split #to split the dataset for training and testing
from sklearn.neighbors import KNeighborsClassifier  # for K nearest neighbours
from sklearn import svm  #for Support Vector Machine (SVM) Algorithm
from sklearn import metrics #for checking the model accuracy
from sklearn.tree import DecisionTreeClassifier #for using Decision Tree Algoithm
from sklearn.ensemble import RandomForestClassifier
iris = pd.read_excel("D:\BehavioralAuth\IDNew_12.xlsx")
#iris = pd.read_excel("D:\Iris Code -- Machine Learning 3\DataRecordMovementToSpeed.xlsx")
vrRecognition = iris[['Angle(°)','BodyMotion(m)','Speed(m/s)','HeadsetY(m)','HeadAngularVelocity(rad/s)','Participant']]

train_p,test_p=train_test_split(vrRecognition,test_size=0.3,random_state=0)
train_x_p=train_p[['Angle(°)','BodyMotion(m)','Speed(m/s)','HeadsetY(m)','HeadAngularVelocity(rad/s)']]
train_y_p=train_p.Participant
test_x_p=test_p[['Angle(°)','BodyMotion(m)','Speed(m/s)','HeadsetY(m)','HeadAngularVelocity(rad/s)']]
test_y_p=test_p.Participant

model = RandomForestClassifier(random_state=42)
model.fit(train_x_p,train_y_p) 
prediction=model.predict(test_x_p) 
print('The accuracy of the RandomForest is:',metrics.accuracy_score(prediction,test_y_p))

model=DecisionTreeClassifier()
model.fit(train_x_p,train_y_p)
prediction=model.predict(test_x_p)
print('The accuracy of the Decision Tree is:',metrics.accuracy_score(prediction,test_y_p))

test = pd.read_excel("D:\BehavioralAuth\DUIExperiment_Evaluation.xlsx")
X = test[['Angle(°)','BodyMotion(m)','Speed(m/s)','HeadsetY(m)','HeadAngularVelocity(rad/s)']]
Y = model.predict(X)
test["predictParticipant"] = Y
test.to_excel("D:\BehavioralAuth\DUIExperiment_Evaluation.xlsx",encoding='utf-8')
print('The accuracy is:',metrics.accuracy_score(test.Participant,test.predictParticipant))
