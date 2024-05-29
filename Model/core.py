
from .DAFNet.config import JsonConfig
import os
import joblib

class Model():

    def __init__(self, hparamTxt, datetime):
        
        """
        Args:
        initialize hparameters & scaler 

        """
        #args = docopt(__doc__)
        self.hparams = hparamTxt #"hparams/preferred/20230529_2312_locomotion.json" # 
        self.hparams = JsonConfig(self.hparams)

        date = str(datetime.datetime.now())
        date = date[:date.rfind(":")].replace("-", "")\
                                        .replace(":", "")\
                                        .replace(" ", "_")
        self.log_dir = os.path.join(self.hparams.Dir.log_root, "log_" + date)
        self.test_dir = self.hparams.Dir.data_dir
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        """
        Scaler
        """
        scaler_root = self.hparams.Dir.scaler_dir
        self.scaler = joblib.load(f'{scaler_root}/mixamo.pkl')

    def buildmodel(self, builder):
        """
        Build Model
        """
        # descriptor clustering network 
        is_training = self.hparams.Infer.pre_trained == ""
        x_dim = self.hparams.Data.pose_features
        sf_dim = x_dim*10 + 3*(10 + 1) + 24 + 1620
        space_dim = 630
        # building models
        if self.hparams.Train.model == "moglow":
            print('moglow')
            built = builder.build(space_dim,sf_dim,self.hparams,is_training)
        
        return built


        