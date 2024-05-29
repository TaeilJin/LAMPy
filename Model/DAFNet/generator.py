import torch
import numpy as np

class GenerateFunc():

    def __init__(self):
        print('DAFNet Generate Function')

    def inv_standardize(self,data, scaler):      
        shape = data.shape
        flat = data.reshape((shape[0]* shape[1]),shape[2])
        scaled = scaler.inverse_transform(flat).reshape(shape)
        return scaled   

    def prepare_cond(self,jt_data, ctrl_data):
        nn,seqlen,n_feats = jt_data.shape
        
        jt_data = jt_data.reshape((nn, seqlen*n_feats))
        nn,seqlen,n_feats = ctrl_data.shape
        ctrl_data = ctrl_data.reshape((nn, seqlen*n_feats))
        cond = torch.from_numpy(np.expand_dims(np.concatenate((jt_data,ctrl_data),axis=1), axis=-1))
        return cond

    def generate_sample_withRef(self,graph, joints_all, vel_all, len_all,\
                                            scaler,z_val=0.0, eps_std=1.0, step=0, counter=0):
        print("generate_sample")
        graph.eval()
        
        with torch.no_grad():
            autoreg_all = joints_all.copy()
            control_all = vel_all.copy()
            len_all = len_all.copy()
            
            seqlen = 10
            
            # Initialize the pose sequence with ground truth test data
            
            # Initialize the lstm hidden state
            if hasattr(graph, "module"):
                graph.module.init_lstm_hidden()
            else:
                graph.init_lstm_hidden()
                
            nn,n_timesteps,n_feats = autoreg_all.shape
            sampled_all = np.zeros((nn, n_timesteps, n_feats))
            reference_all = np.zeros((nn, n_timesteps, n_feats))
            autoreg = np.zeros((nn, seqlen, n_feats), dtype=np.float32) #initialize from a mean pose
            sampled_all[:,:seqlen,:] = autoreg
            
            # sample from Moglow
            sampled_z_label = torch.zeros((1,66,1))
        
            eps_std = 1
            sampled_z_label = torch.normal(mean=torch.zeros(sampled_z_label.shape),
                            std=torch.ones(sampled_z_label.shape) * eps_std)

            time_sum = 0
        # Loop through control sequence and generate new data
            for i in range(0,control_all.shape[1]-seqlen):
                control = control_all[:,i:(i+seqlen+1),:]
                refpose = autoreg_all[:,(i+seqlen):(i+seqlen+1),:]

                # prepare conditioning for moglow (control + previous poses)
                descriptor = self.prepare_cond(autoreg.copy(), control.copy())
                #env = env_all[:,(i+seqlen):(i+seqlen+1),:]
                #env = torch.from_numpy(np.swapaxes(env,1,2)).to(self.data_device)
                len = len_all[:,(i+seqlen):(i+seqlen+1),:]
                len = torch.from_numpy(np.swapaxes(len,1,2))

                # condition (vel + env)
                cond = control_all[:,(i+seqlen):(i+seqlen+1),:]
                cond = torch.from_numpy(np.swapaxes(cond,1,2))
                cond = torch.cat((cond,len),dim=1)

                # descriptor (sequence + env)
                nBatch, nFeatures, nTimesteps = descriptor.shape
                descriptor = torch.cat((descriptor,len),dim=1)
        
                # sample from Moglow
                sampled_z_label = torch.zeros((nBatch,75,1))

                sampled = graph(z=sampled_z_label, cond=descriptor, eps_std=1.0, reverse=True)

                #start = time.time()
                # graph.to('cpu')
                # sampled_z_label = sampled_z_label.to('cpu')
                # cond = cond.to('cpu')
                # ee_cond = ee_cond.to('cpu')
                
                #sampled = graph(z=sampled_z_label, cond=cond, ee_cond = ee_cond, x_head = x_head, x_hand = x_hand, eps_std=eps_std, reverse=True)
                
                #print(" computation time: " , time.time()-start)
                sampled = sampled.cpu().numpy()[:,:,0]

                # store the sampled frame
                sampled_all[:,(i+seqlen),:] = sampled # sampled
                reference_all[:,(i+seqlen),:] = np.swapaxes(refpose,1,2)[:,:,0] # GT
                # update saved pose sequence
                autoreg = np.concatenate((autoreg[:,1:,:].copy(), sampled[:,None,:]), axis=1)
        print("mean of sec time ", time_sum /(control_all.shape[1]-seqlen) )
        
        scaled_all = self.inv_standardize(np.concatenate((sampled_all,vel_all), axis=-1),scaler)
        joints_all_unscaled = scaled_all[:,10:,:75]
        vel_all_unscaled = scaled_all[:,10:,-3:]
        
        ref_scaled_all = self.inv_standardize(np.concatenate((reference_all,vel_all), axis=-1),scaler)
        ref_joints_all_unscaled = ref_scaled_all[:,10:,:75]
        

        return joints_all_unscaled, vel_all_unscaled, ref_joints_all_unscaled