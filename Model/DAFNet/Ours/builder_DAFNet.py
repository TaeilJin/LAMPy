import re, os
import copy
import torch
from collections import defaultdict
from .. import learning_rate_schedule
from ..config import JsonConfig
from .model_DAFNet import DAFNetGen

#from .DAFNet_SAMP import DAFNet_SAMP
#from .models_NA_SAMP_woEE import models_NA_SAMP_woEE

#from .moGlow import Glow_GMM_ENV
#from .models_woHierGlow_GMM_ENV_upperCons import Glow_GMM_ENV_UPPER
from ..utils import load, save, get_proper_device


def build_adam(params, args):
    return torch.optim.Adam(params, **args)


__build_optim_dict = {
    "adam": build_adam
}


def build_Gen(x_channels, cond_channels, hparams, is_training):
    if isinstance(hparams, str):
        hparams = JsonConfig(hparams)
    # get graph and criterions from build function
    graph, optim, lrschedule, criterion_dict = None, None, None, None  # init with None
    cpu, devices = "cpu", None
    get_loss = None
    # 1. build graph and criterion_dict, (on cpu)
    # build and append `device attr` to graph
    if hparams.Train.model =="DAFNet_Gen": # ours wo gmm <-> diversity 영향
        graph = DAFNetGen(x_channels, cond_channels, hparams)
    # elif hparams.Train.model == "models_NA_SAMP":
    #     graph = DAFNet_SAMP(x_channels, cond_channels, hparams)
    # elif hparams.Train.model =="moglow": # 
    #     graph = Glow_GMM_ENV(x_channels,cond_channels,hparams)
    # elif hparams.Train.model == "models_NA_SAMP_woEE":
    #     graph = models_NA_SAMP_woEE(x_channels, cond_channels, hparams)
    
    
    graph.device = hparams.Device.glow
    if graph is not None:
        # get device
        devices = get_proper_device(graph.device)
        graph.device = devices
        graph.to(cpu)
    # 2. get optim (on cpu)
    try:
        if graph is not None and is_training:
            optim_name = hparams.Optim.name
            optim = __build_optim_dict[optim_name](graph.parameters(), hparams.Optim.args.to_dict())
            print("[Builder]: Using optimizer `{}`, with args:{}".format(optim_name, hparams.Optim.args))
            # get lrschedule
            schedule_name = "default"
            schedule_args = {}
            if "Schedule" in hparams.Optim:
                schedule_name = hparams.Optim.Schedule.name
                schedule_args = hparams.Optim.Schedule.args.to_dict()
            if not ("init_lr" in schedule_args):
                schedule_args["init_lr"] = hparams.Optim.args.lr
            assert schedule_args["init_lr"] == hparams.Optim.args.lr,\
                "Optim lr {} != Schedule init_lr {}".format(hparams.Optim.args.lr, schedule_args["init_lr"])
            lrschedule = {
                "func": getattr(learning_rate_schedule, schedule_name),
                "args": schedule_args
            }
    except KeyError:
        raise ValueError("[Builder]: Optimizer `{}` is not supported.".format(optim_name))
    # 3. warm start and move to devices
    if graph is not None:
        # 1. warm start from pre-trained model (on cpu)
        pre_trained = None
        loaded_step = 0
        if is_training:
            if "warm_start" in hparams.Train and len(hparams.Train.warm_start) > 0:
                pre_trained = hparams.Train.warm_start
        else:
            pre_trained = hparams.Infer.pre_trained
        if pre_trained is not None:
            loaded_step = load(os.path.basename(pre_trained),
                        graph=graph, optim=optim,means=graph.means,variance=graph.vars, criterion_dict=None,
                        pkg_dir=os.path.dirname(pre_trained),
                        device=cpu)
        # 2. move graph to device (to cpu or cuda)
        use_cpu = any([isinstance(d, str) and d.find("cpu") >= 0 for d in devices])
        if use_cpu:
            graph = graph.cpu()
            print("[Builder]: Use cpu to train.")
        else:
            if "data" in hparams.Device:
                data_gpu = hparams.Device.data
                if isinstance(data_gpu, str):
                    data_gpu = int(data_gpu[5:])
            else:
                data_gpu = devices[0]
            # move to first
            graph = graph.cuda(device=devices[0])
            if is_training and pre_trained is not None:
                # note that it is possible necessary to move optim
                if hasattr(optim, "state"):
                    def move_to(D, device):
                        for k in D:
                            if isinstance(D[k], dict) or isinstance(D[k], defaultdict):
                                move_to(D[k], device)
                            elif torch.is_tensor(D[k]):
                                D[k] = D[k].cuda(device)
                    move_to(optim.state, devices[0])
            print("[Builder]: Use cuda {} to train, use {} to load data and get loss.".format(devices, data_gpu))

    return {
        "graph": graph,
        "optim": optim,
        "lrschedule": lrschedule,
        "devices": devices,
        "data_device": data_gpu if not use_cpu else "cpu",
        "loaded_step": loaded_step
    }
