# -*- coding: utf-8 -*-
import os
import h5py
import itertools
import pandas as pd
import numpy as np
import noisereduce as nr
from random import choice
from string import ascii_lowercase

from scipy.stats import kurtosis
from obspy.signal import detrend
from scipy.signal import (
    hilbert,
    butter,
    lfilter,
    filtfilt,
    iirnotch,
    #     iircomb,
    get_window,
    spectrogram,
)
from obspy.signal.trigger import carl_sta_trig

from azure.storage.blob import BlobServiceClient

# from scipy import stats
# from scipy.signal import
# import matplotlib.pyplot as plt
# import pywt


# from filterpy.monte_carlo import systematic_resample
class preprocess:
    """
    Description:
    -------
        소음검사 테스트 파일(3번의 테스트가 통합된)을 처리하기 위한 전처리 함수들
    -------
    """

    def __init__(self, sample_rate=42000, test_no=[1, 2]):
        self.sample_rate = sample_rate
        self.test_no = test_no

    def ReadData(self, y, header="infer", names=None, path="", data_format="Raw"):
        filename = y
        if data_format == "csv":
            data = pd.read_csv(filename, names=[0, 1, 2])
        else:
            with h5py.File(filename, mode="r") as f:
                data = pd.DataFrame(f["Raw"][:], columns=[0, 1, 2])

        return data

    def ReadDataBlob(
        self,
        y,
        header="infer",
        names=None,
        path="",
        data_format="Raw",
        container="mc-origin-data-jyt",
        con_string="xxx",
        tmpfilename="tmp",
        tmp_dir="data/tmp/",
        remove_tmp_immediately=True,
    ):

        blobname = y.replace("/" + container + "/", "")
        blob_service_client = BlobServiceClient.from_connection_string(con_string)
        blob_client = blob_service_client.get_blob_client(
            container=container, blob=blobname
        )
        if remove_tmp_immediately is True:
            tmpfilename = tmp_dir + tmpfilename + _random_string_generator(4) + ".h5"
        else:
            tmpfilename = tmpfilename + ".h5"

        with open(tmpfilename, "wb") as my_blob:
            download_stream = blob_client.download_blob()
            my_blob.write(download_stream.readall())

        if data_format == "csv":
            data = pd.read_csv(tmpfilename, names=[0, 1, 2])
        else:
            with h5py.File(tmpfilename, mode="r") as f:
                data = pd.DataFrame(f["Raw"][:], columns=[0, 1, 2])

        if remove_tmp_immediately is True:
            os.remove(tmpfilename)

        return data

    def PreEmphasisRaw(self, y, scale_factor=0.97):
        """
        Previous Steps: ReadData, ScaleData
        """
        data = y.copy()
        tmp = data[2].values
        # x_preemahsized'(t) = x(t) − α × x(t−1)
        data[2] = np.append(tmp[0], tmp[1:] - scale_factor * tmp[:-1])

        return data

    def GetValidInterval(
        self, y, n=None, time_from=0.2, time_to=0.4, sample_rate=42000
    ):
        """
        Previous Steps: ReadData, ScaleData
        """
        if sample_rate is None:
            sample_rate = self.sample_rate
        data = y.copy()

        def get_start_time(x):
            tmp = np.where(x >= 4.00, 1, 0)
            start_time = np.where(tmp - np.append(tmp[1:], 0) == -1)
            if len(start_time[0]) != 3:
                raise NameError("The number of start time should be equal to 3")

            return start_time[0]

        # 전원 On 시점 +0.2 ~ +0.4 지점 추출
        time_start = get_start_time(data[1].values)

        test1 = data[2][
            np.where(
                (data[0] >= data[0][time_start[0]] + time_from)
                & (data[0] <= data[0][time_start[0]] + time_to)
            )[0]
        ]
        test2 = data[2][
            np.where(
                (data[0] >= data[0][time_start[1]] + time_from)
                & (data[0] <= data[0][time_start[1]] + time_to)
            )[0]
        ]
        test3 = data[2][
            np.where(
                (data[0] >= data[0][time_start[2]] + time_from)
                & (data[0] <= data[0][time_start[2]] + time_to)
            )[0]
        ]

        if n is not None:
            if n > len(test1):
                test1 = np.pad(
                    test1, (0, n - len(test1)), "constant", constant_values=0
                )
                test2 = np.pad(
                    test2, (0, n - len(test2)), "constant", constant_values=0
                )
                test3 = np.pad(
                    test3, (0, n - len(test3)), "constant", constant_values=0
                )

            data = np.array([test1[:n], test2[:n], test3[:n]])
        else:
            data = np.array([test1.values, test2.values, test3.values])

        if len(data.shape) != 2:
            raise NameError("Error")

        return data[self.test_no]

    def GetExactInterval(
        self, y, n=None, time_from=0, time_to=0.1, sample_rate=42000, rep_n=3
    ):
        """
        Previous Steps: ReadData, ScaleData
        """
        if sample_rate is None:
            sample_rate = self.sample_rate
        data = y.copy()

        interval = data[2][np.where((data[0] >= time_from) & (data[0] <= time_to))[0]]
        data = np.array([interval[:n].values for i in range(rep_n)])

        if len(data.shape) != 2:
            raise NameError("Error")

        return data

    def GetValidIntervalDataFrame(
        self, y, n=None, time_from=0.2, time_to=0.4, sample_rate=42000
    ):
        """
        Previous Steps: ReadData, ScaleData
        """
        if sample_rate is None:
            sample_rate = self.sample_rate
        data = y.copy().reset_index(drop=True)

        def get_start_time(x):
            tmp = np.where(x >= 4.00, 1, 0)
            start_time = np.where(tmp - np.append(tmp[1:], 0) == -1)
            if len(start_time[0]) != 3:
                raise NameError("The number of start time should be equal to 3")

            return start_time[0]

        # 전원 On 시점 +0.2 ~ +0.4 지점 추출
        time_start = get_start_time(data[1].values)

        interval1 = np.where(
            (data[0] >= data[0][time_start[0]] + time_from)
            & (data[0] <= data[0][time_start[0]] + time_to)
        )[0]
        interval2 = np.where(
            (data[0] >= data[0][time_start[1]] + time_from)
            & (data[0] <= data[0][time_start[1]] + time_to)
        )[0]
        interval3 = np.where(
            (data[0] >= data[0][time_start[2]] + time_from)
            & (data[0] <= data[0][time_start[2]] + time_to)
        )[0]

        data = data.iloc[
            np.concatenate((interval1, interval2, interval3), axis=None), :
        ]

        return data


class signal:
    def __init__(self, sample_rate=42000):
        self.sample_rate = sample_rate

    def Detrend(self, y, order=3):
        Y = detrend.polynomial(y, order)

        return Y

    def Window(self, y):
        w = get_window("hann", Nx=len(y), fftbins=True)
        Y = y * w

        return Y

    def Spectrum(self, y, sample_rate=None, n_padding=None, dB=False, dBref=1e-6):
        if sample_rate is None:
            sample_rate = self.sample_rate
        if n_padding is not None:
            n = n_padding
        else:
            n = len(y)  # 전체 데이터 길이
        k = np.arange(n)  # 순서대로 나래비
        T = n / sample_rate  # 수집 시간 계산
        freq = k / T  # 주파수 index 계산
        freq = freq[: (n // 2)]  # 주파수 index 앞에서부터 반만 가져옴

        Y = np.fft.fft(y, n)
        Y = 2 / n * np.abs(Y[: (n // 2)])  # 절대값하여 복소수 제거한 후, 진폭의 크기를 나타내도록 * 2를 한다.

        if dB is True:
            Y = 20 * np.log10(abs(Y) / dBref)  # dB로 변환
        else:
            Y = abs(Y)

        return Y, freq

    def Cepstrum(self, y, sample_rate=None, n_padding=None):
        if sample_rate is None:
            sample_rate = self.sample_rate
        if n_padding is not None:
            n = n_padding
        else:
            n = len(y)  # 전체 데이터 길이
        k = np.arange(n)  # 순서대로 나래비
        quef = k / sample_rate  # 주파수 index 계산
        quef = quef[range(int(n / 2))]  # 주파수 index 앞에서부터 반만 가져옴

        Y = np.fft.ifft(np.log(np.abs(np.fft.fft(y, n)))).real
        Y = Y[range(int(n / 2))]

        return Y, quef

    def Spectrogram(
        self,
        y,
        sample_rate=None,
        nperseg=None,
        noverlap=None,
        nfft=None,
        scaling="spectrum",
        mode="psd",
        window="hann",
    ):
        if sample_rate is None:
            sample_rate = self.sample_rate
        window = get_window(window, nperseg)
        spectro = spectrogram(
            x=y,
            fs=sample_rate,
            noverlap=noverlap,
            nfft=nfft,
            nperseg=nperseg,
            scaling=scaling,
            window=window,
            mode=mode,
        )

        return np.array(spectro, dtype="object")

    def Envelope(self, y, square=False):
        if square is True:
            Y = np.power(np.abs(hilbert(y)), 2)
        else:
            Y = np.abs(hilbert(y))

        return Y

    def FilterHighpass(self, y, lowcut=10000, order=9, sample_rate=None):
        if sample_rate is None:
            sample_rate = self.sample_rate

        def butter_highpass(lowcut, fs, order=5):
            nyq = 0.5 * fs
            low = lowcut / nyq
            b, a = butter(order, low, btype="high")
            return b, a

        def butter_highpass_filter(data, lowcut, fs, order=5):
            b, a = butter_highpass(lowcut, fs, order=order)
            y = filtfilt(b, a, data)
            return y

        Y = butter_highpass_filter(data=y, lowcut=lowcut, fs=sample_rate, order=order)
        return Y

    def FilterLowpass(self, y, highcut=10000, order=5, sample_rate=None):
        if sample_rate is None:
            sample_rate = self.sample_rate

        def butter_lowpass(highcut, fs, order=5):
            nyq = 0.5 * fs
            high = highcut / nyq
            b, a = butter(order, high, btype="low")
            return b, a

        def butter_lowpass_filter(data, highcut, fs, order=5):
            b, a = butter_lowpass(highcut, fs, order=order)
            y = filtfilt(b, a, data)
            return y

        Y = butter_lowpass_filter(data=y, highcut=highcut, fs=sample_rate, order=order)
        return Y

    def FilterBandpass(
        self,
        y,
        lowcut=1000,
        highcut=2000,
        order=9,
        sample_rate=None,
        freq=None,
        lower=0.095,
        upper=1.05,
    ):
        """
        band pass filter

        Description:
        -------
            정해진 주파수 구간의 신호만 추출하는 필터
        -------

        Parameters:
        -------
            y: numpy array, shape=[N]
                wave 데이터
            lowcut: int
                주파수 하한
            highcut: int
                주파수 상한
            order: int
                필터의 order (order가 클수록 필터의 경사가 급해짐)
            sample rate: int
                샘플 레이트
            freq: 관심 주파수
                특정 주파수의 +- 구간을 정할 경우 사용
            lower: freq 기준 하한
                lower * freq = lowcut
            upper: freq 기준 상한
                upper * freq = highcut
        -------
        """
        if freq is not None:
            lowcut = freq * lower
            highcut = freq * upper

        if sample_rate is None:
            sample_rate = self.sample_rate

        def butter_bandpass(lowcut, highcut, fs, order):
            nyq = 0.5 * fs
            low = lowcut / nyq
            high = highcut / nyq
            b, a = butter(order, [low, high], btype="band")
            return b, a

        def butter_bandpass_filter(data, lowcut, highcut, fs, order):
            b, a = butter_bandpass(lowcut, highcut, fs, order=order)
            # y = lfilter(b, a, data)
            y = filtfilt(b, a, data)
            return y

        Y = butter_bandpass_filter(
            data=y, lowcut=lowcut, highcut=highcut, fs=sample_rate, order=order
        )
        return Y

    def FilterNotch(
        self, y, center=None, interval=3, sample_rate=None, normalized=False
    ):
        """
        Notch Filter
        sample_rate: Sample frequency (Hz)
        center: Frequency to be removed from signal (Hz)
        interval: Quality factor
        """
        if sample_rate is None:
            sample_rate = self.sample_rate

        center = center / (sample_rate / 2) if normalized else center
        # Design Filter
        b, a = iirnotch(w0=center, Q=center / interval, fs=sample_rate)

        # Frequency response
        Y = lfilter(b, a, y)

        return Y

    #     def FilterComb(
    #         self,
    #         y,
    #         center=None,
    #         interval=3,
    #         ftype="notch",
    #         sample_rate=None,
    #         normalized=False,
    #     ):
    #         """
    #         Comb Filter
    #         sample_rate: Sample frequency (Hz)
    #         center: Frequency to be removed from signal (Hz)
    #         interval: Quality factor
    #         """
    #         if sample_rate is None:
    #             sample_rate = self.sample_rate

    #         center = center / (sample_rate / 2) if normalized else center
    #         # Design Filter
    #         b, a = iircomb(w0=center, Q=center / interval, ftype=ftype, fs=sample_rate)

    #         # Frequency response
    #         Y = lfilter(b, a, y)

    #         return Y

    def FilterActiveNoiseCanceling(
        self,
        x,
        y,
        n_grad_freq=2,
        n_grad_time=4,
        n_fft=2048,
        win_length=2048,
        hop_length=512,
        n_std_thresh=1.5,
        prop_decrease=1.0,
        pad_clipping=True,
        verbose=False,
    ):
        """
        x: noise
        y: siganl
        n_grad_freq (int): how many frequency channels to smooth over with the mask.
        n_grad_time (int): how many time channels to smooth over with the mask.
        n_fft (int): number audio of frames between STFT columns.
        win_length (int): Each frame of audio is windowed by `window()`. The window will be of length `win_length` and then padded with zeros to match `n_fft`..
        hop_length (int):number audio of frames between STFT columns.
        n_std_thresh (int): how many standard deviations louder than the mean dB of the noise (at each frequency level) to be considered signal
        prop_decrease (float): To what extent should you decrease noise (1 = all, 0 = none)
        pad_clipping (bool): Pad the signals with zeros to ensure that the reconstructed data is equal length to the data
        use_tensorflow (bool): Use tensorflow as a backend for convolution and fft to speed up computation
        verbose (bool): Whether to plot the steps of the algorithm

        """
        Y = nr.reduce_noise(
            audio_clip=y,
            noise_clip=x,
            n_grad_freq=n_grad_freq,
            n_grad_time=n_grad_time,
            n_fft=n_fft,
            win_length=win_length,
            hop_length=hop_length,
            n_std_thresh=n_std_thresh,
            prop_decrease=prop_decrease,
            pad_clipping=pad_clipping,
            verbose=verbose,
        )

        return Y


class feature:
    def __init__(self, sample_rate=42000):
        self.sample_rate = sample_rate

    def Mean(self, y):
        return np.mean(y)

    def Crest(self, y):
        signal = y
        pk = (np.max(signal) - np.min(signal)) / 2
        rms = np.sqrt(np.sum(signal ** 2) / len(signal))
        crest = pk / rms
        return crest

    def MaxRMS(self, y):
        signal = y
        rms = np.sqrt(np.sum(signal ** 2) / len(signal))
        max_rms = np.max(signal) / rms
        return max_rms

    def Kurtosis(self, y):
        signal = y
        kurt = kurtosis(signal)
        return kurt

    def RMS(self, y):
        signal = y
        rms = np.sqrt(np.sum(signal ** 2) / len(signal))
        return rms

    def Peak(self, y):
        signal = y
        pk = (np.max(signal) - np.min(signal)) / 2
        return pk

    def AggFrequency(self, x, y, freq=None, bandwidth=None, agg_fun="max"):
        """
        x: frequency
        y: amplitude
        """
        if agg_fun == "max":
            func = np.max
        if (agg_fun == "avg") | (agg_fun == "mean"):
            func = self.Mean
        if agg_fun == "median":
            func = np.median
        if agg_fun == "min":
            func = np.min
        if agg_fun == "crest":
            func = self.Crest
        if agg_fun == "kurtosis":
            func = self.Kurtosis
        if agg_fun == "rms":
            func = self.RMS
        if agg_fun == "peak":
            func = self.Peak
        if agg_fun == "maxrms":
            func = self.MaxRMS

        aggfreq = func(y[(x >= (freq - bandwidth / 2)) & (x <= (freq + bandwidth / 2))])

        return aggfreq

    def AggQuefrency(self, x, y, quef=None, bandwidth=0.0006, agg_fun="max"):
        """
        This function is same to MAXFrequency
        x: quefrency
        y: amplitude
        """
        if agg_fun == "max":
            func = np.max
        if (agg_fun == "avg") | (agg_fun == "mean"):
            func = self.Mean
        if agg_fun == "median":
            func = np.median
        if agg_fun == "min":
            func = np.min
        if agg_fun == "crest":
            func = self.Crest
        if agg_fun == "kurtosis":
            func = self.Kurtosis
        if agg_fun == "rms":
            func = self.RMS
        if agg_fun == "peak":
            func = self.Peak
        if agg_fun == "maxrms":
            func = self.MaxRMS

        aggquef = func(y[(x >= (quef - bandwidth / 2)) & (x <= (quef + bandwidth / 2))])

        return aggquef

    def Energy(self, y, method="square"):
        if method == "square":
            fun = np.square
        elif method == "abs":
            fun = np.abs

        energy = np.mean(fun(y)) / self.sample_rate
        return energy

    def WindowedAggregation(
        self, y, n_kernel=204, stride=102, pool_mode="rms", agg_fun="max"
    ):

        if agg_fun == "max":
            func = np.max
        if (agg_fun == "avg") | (agg_fun == "mean"):
            func = self.Mean
        if agg_fun == "median":
            func = np.median
        if agg_fun == "min":
            func = np.min
        if agg_fun == "crest":
            func = self.Crest
        if agg_fun == "kurtosis":
            func = self.Kurtosis
        if agg_fun == "rms":
            func = self.RMS
        if agg_fun == "peak":
            func = self.Peak

        res = func(_pool1d(y, n_kernel, stride, pool_mode))

        return res

    def Triggers(
        self,
        y,
        threshold=0,
        n_trigger_min=1,
        delete_mask_frame_index=0,
        nsta=0.001,
        nlta=0.01,
        ratio=0,
        quiet=0.05,
    ):
        nsta = int(nsta * self.sample_rate)
        nlta = int(nlta * self.sample_rate)
        ratio = ratio
        quiet = quiet

        cft = carl_sta_trig(y, nsta, nlta, ratio, quiet)
        condition = np.where(cft >= threshold, True, False)
        n_trigger = [
            sum(1 for _ in group) for key, group in itertools.groupby(condition) if key
        ]

        return len(n_trigger)

    def CorrSpectrums(self, x, y):
        """
        x: (magnitude, frequency)
        y: (magnitude, frequency)
        """
        x = x[0]
        y = y[0]
        corr = np.corrcoef(x, y)[0, 1]

        return corr

    def CorrSpectrograms(self, x, y):
        """
        x: (time, frequency, magnitude)
        y: (time, frequency, magnitude)
        """
        x = np.mean(x[2], axis=1)
        y = np.mean(y[2], axis=1)
        corr = np.corrcoef(x, y)[0, 1]

        return corr


def _pool1d(a, n_kernel, stride, pool_mode="rms", padding=0):
    """
    1D Pooling

    Parameters:
        A: input 1D array
        kernel_size: int, the size of the window
        stride: int, the stride of the window
        padding: int, implicit zero paddings on both sides of the input
        pool_mode: string, 'max' or 'avg'
    """

    # Padding
    a = np.pad(a, padding, mode="constant")

    # Window view of A

    output_shape = ((len(a) - n_kernel) // stride + 1,)
    kernel_size = (n_kernel,)

    a_w = np.lib.stride_tricks.as_strided(
        a,
        shape=output_shape + kernel_size,
        strides=(stride * a.strides[0],) + a.strides,
    )
    a_w = a_w.reshape(-1, *kernel_size)

    # Return the result of pooling
    if pool_mode == "max":
        return a_w.max(axis=1).reshape(output_shape)
    elif pool_mode == "avg":
        return a_w.mean(axis=1).reshape(output_shape)
    elif pool_mode == "std":
        return a_w.std(axis=1).reshape(output_shape)
    elif pool_mode == "rms":
        return np.sqrt(np.square(a_w).mean(axis=1)).reshape(output_shape)


def _random_string_generator(strlen=4):
    """일정 길이의 랜덤한 string을 생성한다."""
    lis = list(ascii_lowercase)
    return "".join(choice(lis) for _ in range(strlen))
