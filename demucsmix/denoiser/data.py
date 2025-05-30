# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
# author: adefossez, adiyoss, balkce

import json
import logging
import os
import re

from .audio import Audioset

logger = logging.getLogger(__name__)

class NoisyCleanSet:
    def __init__(self, json_file, length=None, stride=None,
                 pad=True, sample_rate=None):
        """__init__.

        :param json_file: json file that points to the audio files
        :param length: maximum sequence length
        :param stride: the stride used for splitting audio sequences
        :param pad: pad the end of the sequence with zeros
        :param sample_rate: the signals sampling rate
        """
        
        with open(json_file, 'r') as f:
            db = json.load(f)
        
        kw = {'length': length, 'stride': stride, 'pad': pad, 'sample_rate': sample_rate}
        self.noisy_set = Audioset(db, set_type="noisy", **kw)
        self.clean_set = Audioset(db, set_type="clean", **kw)
        self.interf_set = Audioset(db, set_type="interf", **kw)
        
        assert len(self.clean_set) == len(self.noisy_set)

    def __getitem__(self, index):
        return self.noisy_set[index], self.clean_set[index], self.interf_set[index]

    def __len__(self):
        return len(self.noisy_set)
