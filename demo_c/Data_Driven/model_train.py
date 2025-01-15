import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd
from sklearn.model_selection import train_test_split

# 判断是否有 GPU 可用
if torch.cuda.is_available():
    device = torch.device('cuda:0')
    print('Using CUDA 11.8')
else:
    device = torch.device('cpu')
    print('Using CPU')

# # 读取数据
# train_x = pd.read_csv('train_x.csv').values
# train_y = pd.read_csv('train_y.csv').values
# test_x = pd.read_csv('test_x.csv').values
# test_y = pd.read_csv('test_y.csv').values
data = pd.read_csv('data.csv').values
train_data, test_data = train_test_split(data, test_size=0.2, random_state=42)

train_x = train_data[:, 0:3]
train_y = train_data[:, 3:6]
test_x = test_data[:, 0:3]
test_y = test_data[:, 3:6]

# 转换为张量
train_x = torch.tensor(train_x, dtype=torch.float32)
train_y = torch.tensor(train_y, dtype=torch.float32)
test_x = torch.tensor(test_x, dtype=torch.float32)
test_y = torch.tensor(test_y, dtype=torch.float32)

# 将数据移动到相应设备上
train_x = train_x.to(device)
train_y = train_y.to(device)
test_x = test_x.to(device)
test_y = test_y.to(device)

# 创建数据加载器
train_dataset = TensorDataset(train_x, train_y)
train_loader = DataLoader(train_dataset, batch_size=1024, shuffle=True)


# 定义模型
class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.fc1 = nn.Linear(3, 12)
        self.fc2 = nn.Linear(12, 8)
        self.fc3 = nn.Linear(8, 10)
        self.fc4 = nn.Linear(10, 12)
        # self.fc5 = nn.Linear(24, 12)
        self.fc6 = nn.Linear(12, 3)

    def forward(self, x):
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = torch.tanh(self.fc3(x))
        x = torch.tanh(self.fc4(x))
        # x = torch.tanh(self.fc5(x))
        x = self.fc6(x)
        return x


model = Net()
model.to(device)


# 初始化模型参数
def init_weights(m):
    if type(m) == nn.Linear:
        torch.nn.init.kaiming_normal_(m.weight)
        m.bias.data.fill_(0.01)


model.apply(init_weights)

# 定义损失函数和优化器
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.02)

# 记录最佳损失值和连续未改进的轮数
best_loss = float('inf')
patience = 10
no_improve = 0

i = 0

# 训练模型
for epoch in range(int(1e7)):
    # 训练一个 epoch
    model.train()
    running_loss = 0.0
    for i, data in enumerate(train_loader):
        inputs, labels = data
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()

        # 梯度裁剪
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1)

        optimizer.step()
        running_loss += loss.item()

    # 检查是否需要停止训练
    if running_loss < best_loss:
        best_loss = running_loss
        no_improve = 0
    else:
        no_improve += 1
        if no_improve >= patience:
            break

    # 打印训练信息
    # print(f'Epoch: {epoch + 1}, Train Loss: {running_loss / len(train_loader)}')

    with open("..\\..\\demo_c\\Data_Driven\\loss.txt", 'w') as f:
        i += 1
        f.write(str([i, running_loss / len(train_loader)]))
# 保存模型参数
torch.save(model.state_dict(), 'model.pth')

# # 加载模型参数
# model.load_state_dict(torch.load('model.pth'))
#
# # 测试模型
# with torch.no_grad():
#     outputs = model(test_x)
#     print(outputs)
#     loss = criterion(outputs, test_y)
#     print(f'Test Loss: {loss.item()}')

# for i, data in enumerate(train_loader):
#     if i < 1:
#         inputs, labels = data
#         print(len(inputs))
#         print(inputs.shape)
#         print(len(labels))
#         print(labels.shape)
#         outputs = model(inputs)
#         # print(labels)
#         # print(outputs)
#         loss = criterion(outputs, labels)
#         # print(loss)

# print("{:.7f}".format(float(test_x[0, 0])))
# print("{:.7f}".format(float(test_x[0, 1])))
# print("{:.7f}".format(float(test_x[0, 2])))
# print("{:.7f}".format(float(test_x[1, 0])))
# print("{:.7f}".format(float(test_x[1, 1])))
# print("{:.7f}".format(float(test_x[1, 2])))
# print("--------------")
# print(test_x[0])
# print(model(test_x[0]))
# print("{:.7f}".format(float((model(test_x[0]))[0, 0])))
# print("{:.7f}".format(float((model(test_x[0]))[0, 1])))
# print("{:.7f}".format(float((model(test_x[0]))[0, 2])))
# print("{:.7f}".format(float((model(test_x[1]))[1, 0])))
# print("{:.7f}".format(float((model(test_x[1]))[1, 1])))
# print("{:.7f}".format(float((model(test_x[1]))[1, 2])))
