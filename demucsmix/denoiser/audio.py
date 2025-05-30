# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
# author: adefossez, balkce

from collections import namedtuple
import json
from pathlib import Path
import math
import random
import os
import sys

import torch
import torchaudio
from torch.nn import functional as F

from .dsp import convert_audio

Info = namedtuple("Info", ["length", "sample_rate", "channels"])

def combine_interf (signal,interf):
    return torch.cat((signal,interf),2)

def extract_interf (signal_w_interf):
    sig_len = int(signal_w_interf.shape[2]/2) #assuming clean and interf are of the same length
    signal = signal_w_interf[:,:,:sig_len]
    interf = signal_w_interf[:,:,sig_len:]
    return signal, interf


def get_clean_path(path):
    info_file_path = path.replace(".wav", ".txt" )
    
    info_file = open(info_file_path,"r")
    clean_path = info_file.readline().rstrip() #first line in txt file is the clean WAV file
    embed_path = info_file.readline().rstrip() #second line in txt file is the embed WAV file (ignored for this strategy)
    interf_path = info_file.readline().rstrip() #third line in txt file is the interf WAV file
    mic_path = info_file.readline().rstrip() #fourth line in txt file is the mic WAV file
    info_file.close()
    
    assert os.path.exists(clean_path), clean_path+"does not exist."
    assert os.path.exists(interf_path), interf_path+"does not exist."
    assert os.path.exists(mic_path), interf_path+"does not exist."
    
    return clean_path, interf_path, mic_path

def get_info(path):
    info = torchaudio.info(path)
    if hasattr(info, 'num_frames'):
        # new version of torchaudio
        return Info(info.num_frames, info.sample_rate, info.num_channels)
    else:
        siginfo = info[0]
        return Info(siginfo.length // siginfo.channels, siginfo.rate, siginfo.channels)


def find_audio_files(path, exts=[".wav"], progress=True):
    audio_files = []
    for root, folders, files in os.walk(path, followlinks=True):
        for file in files:
            file = Path(root) / file
            if file.suffix.lower() in exts:
                audio_files.append(str(file.resolve()))
    meta = []
    
    for idx, file in enumerate(audio_files):
        info = get_info(file)
        clean_audio_path, interf_audio_path, mic_audio_path = get_clean_path(file)
        info_clean = get_info(clean_audio_path)
        
        this_length = min(info.length,info_clean.length)
        meta.append((file, clean_audio_path, interf_audio_path, mic_audio_path, this_length))
        if progress:
            print(format((1 + idx) / len(audio_files), " 3.1%"), end='\r')
    if progress:
      print("")
    meta.sort()
    return meta

class Audioset:
    def __init__(self, files=None, length=None, stride=None,
                 pad=True, with_path=False, sample_rate=None,
                 channels=None, convert=False, set_type=""):
        """
        files should be a list [(file, clean_file, interf_file, clean_file_length)]
        """
        self.files = files
        self.num_examples = []
        self.length = length
        self.stride = stride or length
        self.with_path = with_path
        self.sample_rate = sample_rate
        self.channels = channels
        self.convert = convert
        self.set_type = set_type
        
        for f_i,(noisy_file, clean_file, interf_file, mic_file, file_length) in enumerate(self.files):
            if (self.set_type == "clean"):
                file = clean_file
            elif (self.set_type == "noisy" or self.with_path == True):
                file = noisy_file
            elif (self.set_type == "interf"):
                file = interf_file
            else:
                assert False, self.set_type+" is not a valid type of Audioset with the argument with_path==False"
            
            if length is None:
                examples = 1
            elif file_length < length:
                examples = 1 if pad else 0
            elif pad:
                examples = int(math.ceil((file_length - self.length) / self.stride) + 1)
                
                #if file_length >= int(1.1*length):
                #    file_length = int(1.1*length)
                #    self.files[f_i][3] = file_length
                #    examples = int(math.ceil((file_length - self.length) / self.stride) + 1)
                #else:
                #    examples = 1
            else:
                examples = (file_length - self.length) // self.stride + 1
            self.num_examples.append(examples)

    def __len__(self):
        return sum(self.num_examples)

    def __getitem__(self, index):
        start_index = index
        for (noisy_file, clean_file, interf_file, mic_file, _), examples in zip(self.files, self.num_examples):
            if index >= examples:
                index -= examples
                continue
            
            if (self.set_type == "clean"):
                file = clean_file
            elif (self.set_type == "noisy" or self.with_path == True):
                file = noisy_file
            elif (self.set_type == "interf"):
                file = interf_file
            else:
                assert False, self.set_type+" is not a valid type of Audioset with the argument with_path==False"
            
            out = self.load_audio(file, index)
            
            if self.with_path:
                clean = self.load_audio(clean_file, index)
                interf,sr = torchaudio.load(str(interf_file))
                mic,sr = torchaudio.load(str(mic_file))
                return out, clean, interf, file, mic
            else:
                return out

    def load_audio(self, audio_path, index):
        num_frames = 0
        offset = 0
        if self.length is not None:
            offset = self.stride * index
            num_frames = self.length
        
        out, sr = torchaudio.load(str(audio_path),
                                  frame_offset=offset,
                                  num_frames=num_frames or -1)
        
        target_sr = self.sample_rate or sr
        target_channels = self.channels or out.shape[0]
        if self.convert:
            out = convert_audio(out, sr, target_sr, target_channels)
        else:
            if sr != target_sr:
                raise RuntimeError(f"Expected {audio_path} to have sample rate of "
                                   f"{target_sr}, but got {sr}")
            if out.shape[0] != target_channels:
                raise RuntimeError(f"Expected {audio_path} to have sample rate of "
                                   f"{target_channels}, but got {sr}")
        if num_frames:
            out = F.pad(out, (0, num_frames - out.shape[-1]))
        
        return out

if __name__ == "__main__":
    database_path = sys.argv[1]
    json_dir_pah = sys.argv[2]
    database_type = sys.argv[3]
    
    print("Reading audio files from: "+database_path)
    meta = find_audio_files(database_path)
    print("Randomizing order of files.")
    random.shuffle(meta)
    
    if database_type == "train,valid,test":
        train_p = 0.7
        valid_p = 0.2
        
        meta_train_len = int(train_p * len(meta))
        meta_valid_len = int(valid_p * len(meta))
        meta_test_len = len(meta) - meta_train_len - meta_valid_len
        
        print("Total number of samples:           "+str(len(meta)))
        print("Percentage for training samples:   "+str(train_p*100)+"%"+" -> training samples: "+str(meta_train_len))
        print("Percentage for validation samples: "+str(valid_p*100)+"%"+" -> validations samples: "+str(meta_valid_len))
        print("Rest are testing samples:          -> "+str(meta_test_len))
        
        assert meta_test_len > 0, "the percentages for training and validation samples creates a non-valid (>0) number of testing samples"
        
        meta_train = meta[:meta_train_len]
        meta_valid = meta[meta_train_len:meta_train_len+meta_valid_len]
        meta_test = meta[meta_train_len+meta_valid_len:]
        
        print("Storing json files in: "+json_dir_pah)
        train_json = open(json_dir_pah+"/train.json","w")
        json.dump(meta_train, train_json, indent=4)
        train_json.close()
        
        valid_json = open(json_dir_pah+"/valid.json","w")
        json.dump(meta_valid, valid_json, indent=4)
        valid_json.close()
        
        test_json = open(json_dir_pah+"/test.json","w")
        json.dump(meta_test, test_json, indent=4)
        test_json.close()
    elif database_type == "train,valid":
        train_p = 0.9
        
        meta_train_len = int(train_p * len(meta))
        meta_valid_len = len(meta) - meta_train_len
        
        print("Total number of samples:           "+str(len(meta)))
        print("Percentage for training samples:   "+str(train_p*100)+"%"+" -> training samples: "+str(meta_train_len))
        print("Rest are validation samples:          -> "+str(meta_valid_len))
        
        meta_train = meta[:meta_train_len]
        meta_valid = meta[meta_train_len:]
        
        print("Storing json files in: "+json_dir_pah)
        train_json = open(json_dir_pah+"/train.json","w")
        json.dump(meta_train, train_json, indent=4)
        train_json.close()
        
        valid_json = open(json_dir_pah+"/valid.json","w")
        json.dump(meta_valid, valid_json, indent=4)
        valid_json.close()
    elif database_type == "valid,test":
        valid_p = 0.5
        
        meta_valid_len = int(valid_p * len(meta))
        meta_test_len = len(meta) - meta_valid_len
        
        print("Total number of samples:           "+str(len(meta)))
        print("Percentage for validation samples:   "+str(valid_p*100)+"%"+" -> validation samples: "+str(meta_valid_len))
        print("Rest are testing samples:          -> "+str(meta_test_len))
        
        meta_valid = meta[:meta_valid_len]
        meta_test = meta[meta_valid_len:]
        
        print("Storing json files in: "+json_dir_pah)
        valid_json = open(json_dir_pah+"/valid.json","w")
        json.dump(meta_valid, valid_json, indent=4)
        valid_json.close()
        
        test_json = open(json_dir_pah+"/test.json","w")
        json.dump(meta_test, test_json, indent=4)
        test_json.close()
    elif database_type == "test":
        print("Total number of samples (all for testing): "+str(len(meta)))
        
        print("Storing json files in: "+json_dir_pah)
        test_json = open(json_dir_pah+"/test.json","w")
        json.dump(meta, test_json, indent=4)
        test_json.close()
    elif database_type == "train":
        print("Total number of samples (all for training): "+str(len(meta)))
        
        print("Storing json files in: "+json_dir_pah)
        train_json = open(json_dir_pah+"/train.json","w")
        json.dump(meta, train_json, indent=4)
        train_json.close()
    elif database_type == "valid":
        print("Total number of samples (all for validation): "+str(len(meta)))
        
        print("Storing json files in: "+json_dir_pah)
        valid_json = open(json_dir_pah+"/valid.json","w")
        json.dump(meta, valid_json, indent=4)
        valid_json.close()
    else:
        print("Invalid database type: train,valid,test; train,valid; valid,test; train; valid; test")

