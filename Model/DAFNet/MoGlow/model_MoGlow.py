import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from tqdm import tqdm
from .. import thops
from .. import modules
from .. import utils

def nan_throw(tensor, name="tensor"):
        stop = False
        if ((tensor!=tensor).any()):
            print(name + " has nans")
            stop = True
        if (torch.isinf(tensor).any()):
            print(name + " has infs")
            stop = True
        if stop:
            print(name + ": " + str(tensor))
            #raise ValueError(name + ' contains nans of infs')
class conditionEncoder_LSTM(nn.Module):
    def __init__(self,in_channels, cond_channels, hidden_channels, out_channels, num_layers):
        super(conditionEncoder_LSTM,self).__init__()
        #self.SAB = SAB(1,1,2)
        self.env_channels = 2640
        self.out_env_channels = 512
        self.in_channel = in_channels

        self.Linear_ENV = nn.Linear(self.env_channels, self.out_env_channels)
        self.Linear_ENV_relu = nn.ReLU()
        self.Linear_ENV_out = nn.Linear(self.out_env_channels, self.out_env_channels)

        cond_channels = cond_channels - self.env_channels + self.out_env_channels
        self.layer_lstm = modules.LSTM(in_channels + cond_channels, hidden_channels, out_channels, num_layers)
    
    def forward(self, input):
        #environment
        env = input[:,:,self.in_channel:self.in_channel+self.env_channels]
        nBatch, nTimesteps, nFeats = env.shape
        env = env.reshape(-1,nFeats) # (B*T,nFeats)
        env = self.Linear_ENV(env) # (B*T,nDim)
        env = self.Linear_ENV_relu(env)
        env = self.Linear_ENV_out(env)
        env = env.unsqueeze(1).reshape(nBatch,nTimesteps,-1)#(B,Feat,Timesteps)

        input = torch.cat((input[:,:,:self.in_channel],env,input[:,:,self.in_channel+self.env_channels:]),dim=-1)
        output = self.layer_lstm(input)

        return output

def f(in_channels, out_channels, hidden_channels, cond_channels, network_model, num_layers):
    if network_model=="LSTM":
        return modules.LSTM(in_channels + cond_channels, hidden_channels, out_channels, num_layers)
    if network_model=="GRU":
        return modules.GRU(in_channels + cond_channels, hidden_channels, out_channels, num_layers)
    if network_model=="FF":
        return nn.Sequential(
        nn.Linear(in_channels+cond_channels, hidden_channels), nn.ReLU(inplace=False),
        nn.Linear(hidden_channels, hidden_channels), nn.ReLU(inplace=False),
        modules.LinearZeroInit(hidden_channels, out_channels))
    if network_model =="LSTM_ENV":
        return conditionEncoder_LSTM(in_channels, cond_channels, hidden_channels, out_channels, num_layers)
    
class FlowStep(nn.Module):
    FlowCoupling = ["additive", "affine","piecewise"]
    NetworkModel = ["LSTM", "GRU", "FF", "LSTM_ENV"]
    FlowPermutation = {
        "reverse": lambda obj, z, logdet, rev: (obj.reverse(z, rev), logdet),
        "shuffle": lambda obj, z, logdet, rev: (obj.shuffle(z, rev), logdet),
        "invconv": lambda obj, z, logdet, rev: obj.invconv(z, logdet, rev)
    }

    def __init__(self, in_channels, hidden_channels, cond_channels,
                 actnorm_scale=1.0,
                 flow_permutation="invconv",
                 flow_coupling="additive",
                 network_model="LSTM",
                 num_layers=2,
                 LU_decomposed=False):
                 
        # check configures
        assert flow_coupling in FlowStep.FlowCoupling,\
            "flow_coupling should be in `{}`".format(FlowStep.FlowCoupling)
        assert network_model in FlowStep.NetworkModel,\
            "network_model should be in `{}`".format(FlowStep.NetworkModel)
        assert flow_permutation in FlowStep.FlowPermutation,\
            "float_permutation should be in `{}`".format(
                FlowStep.FlowPermutation.keys())
        super().__init__()
        self.flow_permutation = flow_permutation
        self.flow_coupling = flow_coupling
        self.network_model = network_model
        # 1. actnorm
        self.actnorm = modules.ActNorm2d(in_channels, actnorm_scale)
        # 2. permute
        if flow_permutation == "invconv":
            self.invconv = modules.InvertibleConv1x1(
                in_channels, LU_decomposed=LU_decomposed)
        elif flow_permutation == "shuffle":
            self.shuffle = modules.Permute2d(in_channels, shuffle=True)
        else:
            self.reverse = modules.Permute2d(in_channels, shuffle=False)
        # 3. coupling
        if flow_coupling == "additive":
            self.f = f(in_channels // 2, in_channels-in_channels // 2, hidden_channels, cond_channels, network_model, num_layers)
        elif flow_coupling == "affine":
            print("affine: in_channels = " + str(in_channels))
            self.f = f(in_channels // 2, 2*(in_channels-in_channels // 2), hidden_channels, cond_channels, network_model, num_layers)
            print("Flowstep pose mask(63) with cond_channels: " + str(cond_channels))
       
    def init_lstm_hidden(self):
        if self.network_model == "LSTM" or self.network_model == "GRU":
            self.f.init_hidden()
        if self.network_model == "LSTM_ENV":
            self.f.layer_lstm.init_hidden()

    def forward(self, input, cond, logdet=None, reverse=False):
        if not reverse:
            return self.normal_flow(input, cond, logdet)
        else:
            return self.reverse_flow(input, cond, logdet)

    def normal_flow(self, input, cond, logdet):
    
        #assert input.size(1) % 2 == 0
        # 1. actnorm
        #z=input
        z, logdet = self.actnorm(input, logdet=logdet, reverse=False)
        # 2. permute
        z, logdet = FlowStep.FlowPermutation[self.flow_permutation](
            self, z, logdet, False)
        # 3. coupling
        z1, z2 = thops.split_feature(z, "split")
        z1_cond = torch.cat((z1, cond), dim=1)
        if self.flow_coupling == "additive":
            z2 = z2 + self.f(z1_cond)
        elif self.flow_coupling == "affine":        
            h = self.f(z1_cond.permute(0, 2, 1)).permute(0, 2, 1)
            shift, scale = thops.split_feature(h, "cross")
            scale = torch.sigmoid(scale + 2.)+1e-6
            z2 = z2 + shift
            z2 = z2 * scale
            logdet = thops.sum(torch.log(scale), dim=[1, 2]) + logdet
            
        z = thops.cat_feature(z1, z2)
        return z, cond, logdet

    def reverse_flow(self, input, cond, logdet):
        # 1.coupling
        z1, z2 = thops.split_feature(input, "split")
        z1_cond = torch.cat((z1, cond), dim=1)

        if self.flow_coupling == "additive":
            z2 = z2 - self.f(z1_cond)
        elif self.flow_coupling == "affine":
            h = self.f(z1_cond.permute(0, 2, 1)).permute(0, 2, 1)
            shift, scale = thops.split_feature(h, "cross")
            nan_throw(shift, "shift")
            nan_throw(scale, "scale")
            nan_throw(z2, "z2 unscaled")
            scale = torch.sigmoid(scale + 2.)+1e-6
            z2 = z2 / scale
            z2 = z2 - shift
            logdet = -thops.sum(torch.log(scale), dim=[1, 2]) + logdet
        

        z = thops.cat_feature(z1, z2)
        # 2. permute
        z, logdet = FlowStep.FlowPermutation[self.flow_permutation](
            self, z, logdet, True)
        nan_throw(z, "z permute_" + str(self.flow_permutation))
       # 3. actnorm
        z, logdet = self.actnorm(z, logdet=logdet, reverse=True)
        return z, cond, logdet

class SelectLayerUpper(nn.Module):
    def __init__(self):
        super().__init__()
        self.upper_idx = [0,1,2, 
        (1)*3+ 0,(1)*3+ 1,(1)*3+ 2, 
        (2)*3+ 0,(2)*3+ 1,(2)*3+ 2, 
        (3)*3+ 0,(3)*3+ 1,(3)*3+ 2, 
        (4)*3+ 0,(4)*3+ 1,(4)*3+ 2, 
        (5)*3+ 0,(5)*3+ 1,(5)*3+ 2,
        (6)*3+ 0,(6)*3+ 1,(6)*3+ 2, 
        (7)*3+ 0,(7)*3+ 1,(7)*3+ 2, 
        (8)*3+ 0,(8)*3+ 1,(8)*3+ 2, 
        (9)*3+ 0,(9)*3+ 1,(9)*3+ 2,
        (10)*3+0,(10)*3+1,(10)*3+2, 
        (11)*3+0,(11)*3+1,(11)*3+2, 
        (12)*3+0,(12)*3+1,(12)*3+2, 
        (13)*3+0,(13)*3+1,(13)*3+2]
        
        self.dim = len(self.upper_idx)

        # head : 5 left hand: 9 right hand :13
        # vector 의 순서는 head, lh, rh
        self.upper_ee_idx =[
            15,16,17, 
            27,28,29,
            39,40,41,
            (17)*3+0,(17)*3+1,(17)*3+2,
            (21)*3+0,(21)*3+1,(21)*3+2]

    def addElement(self, z, z_u ):
        for ii, index in enumerate(self.upper_idx):
            z[:,index,:] = z_u[:,ii,:] 
        return z
    
    def addEndEffectorElement(self,z, ee_cond):
        # 순서가 Head LH, RH 라는 것을 기억하자
        for ii, index in enumerate(self.upper_ee_idx):
            z[:,index,:] = ee_cond[:,ii,:] 
        return z
    
    def forward(self, input,reverse=False):
        if not reverse:
            self.mask = torch.zeros(input.shape).to(input.device)
            self.mask[:,self.upper_idx,:] = 1
            
            output = input[:,self.upper_idx,:]
            self.masked_pose = input.mul(self.mask)
            
            return output, self.masked_pose
        else:
            self.mask = torch.zeros(input.shape).to(input.device)
            self.mask[:,self.upper_idx,:] = 1

            output = input[:,self.upper_idx,:]
            self.masked_pose = input.mul(self.mask)
            
            return output, self.masked_pose
    
    def getdim(self):
        return self.dim

class FlowNet(nn.Module):
    def __init__(self, x_channels, hidden_channels, cond_channels, K,
                 actnorm_scale=1.0,
                 flow_permutation="invconv",
                 flow_coupling="additive",
                 network_model="LSTM",
                 num_layers=2,
                 LU_decomposed=False):
                 
        super().__init__()
        self.layers = nn.ModuleList()
        self.output_shapes = []
        self.K = K
        N = cond_channels

       
        # flow module upper
        self.layers_u = nn.ModuleList()
        for _ in range(K):
            self.layers_u.append(
                FlowStep(in_channels=x_channels,
                         hidden_channels=hidden_channels,
                         cond_channels=N,
                         actnorm_scale=actnorm_scale,
                         flow_permutation=flow_permutation,
                         flow_coupling=flow_coupling,
                         network_model=network_model,
                         num_layers=2,
                         LU_decomposed=LU_decomposed))
            self.output_shapes.append(
                [-1, x_channels, 1])

       

    def init_lstm_hidden(self):
        for layer in self.layers_u:
            if isinstance(layer, FlowStep):                
                layer.init_lstm_hidden()
  

    def forward(self, z, cond, logdet=0., reverse=False, eps_std=None):
        if not reverse:
            # train x->z
            for layer in self.layers_u:
                z, _, logdet = layer(z, cond, logdet, reverse=False)
            
            
            return z, logdet
        else:
            # train z->x
            for i,layer in enumerate(reversed(self.layers_u)):
                z, _, logdet = layer(z, cond, logdet=0, reverse=True)
            
            return z


class Glow(nn.Module):

    def __init__(self, x_channels, cond_channels, hparams):
        super().__init__()
        self.flow = FlowNet(x_channels=x_channels,
                            hidden_channels=hparams.Glow.hidden_channels,
                            cond_channels=cond_channels,
                            K=hparams.Glow.K,
                            actnorm_scale=hparams.Glow.actnorm_scale,
                            flow_permutation=hparams.Glow.flow_permutation,
                            flow_coupling=hparams.Glow.flow_coupling,
                            network_model=hparams.Glow.network_model,
                            num_layers=hparams.Glow.num_layers,
                            LU_decomposed=hparams.Glow.LU_decomposed)
        self.hparams = hparams
        self.means= None
        # register prior hidden
        num_device = len(utils.get_proper_device(hparams.Device.glow, False))
        assert hparams.Train.batch_size % num_device == 0
        self.z_shape = [hparams.Train.batch_size // num_device, x_channels, 1]
        if hparams.Glow.distribution == "normal":
            self.distribution = modules.GaussianDiag()
        
    def init_lstm_hidden(self):
        self.flow.init_lstm_hidden()

    def forward(self, x=None, cond=None, z=None, 
                eps_std=None, reverse=False):
        if not reverse:
            return self.normal_flow(x, cond)
        else:
            return self.reverse_flow(z, cond,eps_std)

    def normal_flow(self, x, cond):
    
        n_timesteps = thops.timesteps(x)

        logdet = torch.zeros_like(x[:, 0, 0])

        # encode
        z, objective = self.flow(x, cond, logdet=logdet, reverse=False)

        # prior
        objective += self.distribution.logp(z)

        # return
        nll = (-objective) / float(np.log(2.) * n_timesteps)
        return z, nll

    def reverse_flow(self, z, cond, eps_std):
        with torch.no_grad():

            z_shape = self.z_shape
            if z is None:
                z = self.distribution.sample(z_shape, eps_std, device=cond.device)

            x = self.flow(z, cond, eps_std=eps_std, reverse=True)
        return x

    def set_actnorm_init(self, inited=True):
        for name, m in self.named_modules():
            if (m.__class__.__name__.find("ActNorm") >= 0):
                m.inited = inited

    @staticmethod
    def loss_generative(nll):
        # Generative loss
        return torch.mean(nll)
        