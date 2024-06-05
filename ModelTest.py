from Model.core import ModelTest
from Model.DAFNet.GMVAE import builder_GMVAE
from Model.DAFNet.Ours import builder_DAFNet
import datetime

mtest = ModelTest('hparams/hparams.json',datetime)

# descriptor clustering network 
is_training_cond = mtest.hparams.Infer.pre_trained_cond ==""
is_training = mtest.hparams.Infer.pre_trained == ""
x_channels = mtest.hparams.Data.pose_features
cond_channels = mtest.hparams.Data.target_root_features + 2640
descriptor_channels = x_channels*10 + mtest.hparams.Data.target_root_features*11 + 2640

# building models
built_Cond = builder_GMVAE.build_GMVAE(x_channels,descriptor_channels,mtest.hparams,is_training_cond)  
if mtest.hparams.Train.model == "moglow":
    print('moglow')
else:
    built = builder_DAFNet.build_Gen(x_channels,cond_channels,mtest.hparams,is_training)  

# testing data