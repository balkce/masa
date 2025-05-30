# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
# author: adiyoss, balkce

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import logging
import os
import sys

import torch
import torchaudio

from .audio import Audioset, find_audio_files, extract_interf, combine_interf
from . import distrib, pretrained
from .demucs import DemucsStreamer

from .utils import LogProgress

logger = logging.getLogger(__name__)

def add_flags(parser):
    """
    Add the flags for the argument parser that are related to model loading and evaluation"
    """
    pretrained.add_model_flags(parser)
    parser.add_argument('--device', default="cpu")
    parser.add_argument('--dry', type=float, default=0,
                        help='dry/wet knob coefficient. 0 is only denoised, 1 only input signal.')
    parser.add_argument('--num_workers', type=int, default=10)
    parser.add_argument('--streaming', action="store_true",
                        help="true streaming evaluation for Demucs")


parser = argparse.ArgumentParser(
        'denoiser.enhance',
        description="Speech enhancement using Demucs - Generate enhanced files")
add_flags(parser)
parser.add_argument("--out_dir", type=str, default="enhanced",
                    help="directory putting enhanced wav files")
parser.add_argument("--batch_size", default=1, type=int, help="batch size")
parser.add_argument('-v', '--verbose', action='store_const', const=logging.DEBUG,
                    default=logging.INFO, help="more loggging")

group = parser.add_mutually_exclusive_group()
group.add_argument("--db_dir", type=str, default=None,
                   help="directory including noisy wav files")
group.add_argument("--db_json", type=str, default=None,
                   help="json file including database files")


def get_estimate(model, noisy, args):
    torch.set_num_threads(1)
    
    if args.streaming:
        streamer = DemucsStreamer(model, dry=args.dry)
        with torch.no_grad():
            estimate = torch.cat([
                streamer.feed(noisy[0]),
                streamer.flush()], dim=1)[None]
    else:
        with torch.no_grad():
            estimate, noise_estimate = extract_interf(model(noisy))
            noisy_wo_interf, interf = extract_interf(noisy)
            estimate = (1 - args.dry) * estimate + args.dry * noisy_wo_interf
    return estimate


def save_wavs(estimates, noisy_sigs, clean_sigs, filenames, mic_sigs, out_dir, sr=16_000):
    # Write result
    for estimate, noisy, clean, filename, mic in zip(estimates, noisy_sigs, clean_sigs, filenames, mic_sigs):
        #user_dir = os.path.dirname(filename)
        #user = os.path.basename(user_dir)
        #babble_snr_dir = os.path.dirname(user_dir)
        #babble_snr = os.path.basename(babble_snr_dir)
        #babble_num_dir = os.path.dirname(babble_snr_dir)
        #babble_num = os.path.basename(babble_num_dir)
        #noise_snr_dir = os.path.dirname(babble_num_dir)
        #noise_snr = os.path.basename(noise_snr_dir)
        #reverbscale_dir = os.path.dirname(noise_snr_dir)
        #reverbscale = os.path.basename(reverbscale_dir)
        #os.path.join(out_dir, os.path.basename(filename).rsplit(".", 1)[0])
        #fileidname =  os.path.join(out_dir, reverbscale+"--"+noise_snr+"--"+babble_num+"--"+babble_snr+"--"+user)
        
        fileidname = os.path.join(out_dir, os.path.basename(filename).rsplit(".", 1)[0])
        write(noisy, fileidname + "_noisy.wav", sr=sr)
        write(estimate, fileidname + "_enhanced.wav", sr=sr)
        write(clean, fileidname + "_clean.wav", sr=sr)
        write(mic, fileidname + "_mic.wav", sr=sr)


def write(wav, filename, sr=16_000):
    # Normalize audio if it prevents clipping
    wav = wav / max(wav.abs().max().item(), 1)
    torchaudio.save(filename, wav.cpu(), sr)


def get_dataset(args, sample_rate, channels):
    if hasattr(args, 'dset'):
        paths = args.dset.dset
    else:
        paths = args
    if paths.db_json:
        with open(paths.db_json) as f:
            files = json.load(f)
    elif paths.db_dir:
        files = find_audio_files(paths.db_dir)
    else:
        logger.warning(
            "Small sample set was not provided by either db_dir or db_json. "
            "Skipping enhancement.")
        return None
    return Audioset(files, with_path=True,
                    sample_rate=sample_rate, channels=channels, convert=True)


def _estimate_and_save(model, noisy_signals, clean_signals, filenames, mic_signals, out_dir, args):
    estimate = get_estimate(model, noisy_signals, args)
    noisy_wo_interf, interf = extract_interf(noisy_signals)
    save_wavs(estimate, noisy_wo_interf, clean_signals, filenames, mic_signals, out_dir, sr=model.sample_rate)


def enhance(args, model=None, local_out_dir=None):
    # Load model
    if not model:
        model = pretrained.get_model(args).to(args.device)
    model.eval()
    if local_out_dir:
        out_dir = local_out_dir
    else:
        out_dir = args.out_dir

    dset = get_dataset(args, model.sample_rate, model.chin)
    if dset is None:
        return
    loader = distrib.loader(dset, batch_size=1)

    if distrib.rank == 0:
        os.makedirs(out_dir, exist_ok=True)
    distrib.barrier()

    with ProcessPoolExecutor(args.num_workers) as pool:
        iterator = LogProgress(logger, loader, name="Generate enhanced files")
        pendings = []
        for data in iterator:
            # Get batch data
            noisy_signals, clean_signals, interf_signals, filenames, mic_signals = data
            noisy_signals = noisy_signals.to(args.device)
            interf_signals = interf_signals.to(args.device)
            
            noisy_interf = combine_interf(noisy_signals,interf_signals)
            
            if args.device == 'cpu' and args.num_workers > 1:
                pendings.append(
                    pool.submit(_estimate_and_save,
                                model, noisy_interf, clean_signals, filenames, mic_signals, out_dir, args))
            else:
                # Forward
                estimate = get_estimate(model, noisy_interf, args)
                save_wavs(estimate, noisy_signals, clean_signals, filenames, mic_signals, out_dir, sr=model.sample_rate)

        if pendings:
            print('Waiting for pending jobs...')
            for pending in LogProgress(logger, pendings, updates=5, name="Generate enhanced files"):
                pending.result()


if __name__ == "__main__":
    args = parser.parse_args()
    logging.basicConfig(stream=sys.stderr, level=args.verbose)
    logger.debug(args)
    enhance(args, local_out_dir=args.out_dir)
