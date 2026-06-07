# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
from tqdm import tqdm
from multiprocessing import Pool


# Functions for parallel processing
class _Run1(object):
    def __init__(self, steps, input_name, input_values, feature_names, verbose):
        """Initialize the class with 'global' variables"""
        self.pl = Pipeline()
        self.steps = steps
        self.input_name = input_name
        self.input_values = input_values
        self.feature_names = feature_names
        self.verbose = verbose

    def __call__(self, i):
        """Do something with the cursor and data"""
        datum = self.pl._run1(
            steps=self.steps,
            input_name=self.input_name,
            input_value=self.input_values[i],
            feature_names=self.feature_names,
            verbose=self.verbose,
        )
        return datum


class Pipeline:
    """
    pipeline

    Description:
    -------
        여러 전처리를 순차적으로 적용하여 데이터를 변형

        - run: 데이터에 전처리 과정을 적용하고 결과를 생성
        - run_batch: 데이터에 전처리 과정을 적용하고 결과를 생성(run 과 동일한 기능)
                     batch_size로 한번에 메모리에 올릴 데이터 수를 결정.

        Pipeline에 사용 가능한 전처리는 다음의 특징을 가짐
            attributes:
             - name: str (전처리 이름)
             - fetures_names: str (특징 이름. None 가능)
             - previous_steps: list (이전에 실행되어야 할 전처리 이름)
            methods:
             - run1: 하나의 데이터(제품)를 전처리하는 함수
             - run: 하나 이상의 데이터(제품)를 전처리하는 함수
                  - 입력값: previous_steps의 결과가 저장된 dictionary
                      - key: previous_steps의 값
                      - value: 각 step의 전처리 결과 데이터

            Ex)
            class FeatureSpectrum():
                def __init__(self,
                            freq_from=3000,
                            freq_to=6000,
                            dB=False,
                            method='mean',
                            name='FeatureSpectrum',
                            previous_steps=['GenerateSpectrum']):
                    self.name = name
                    self.freq_from=freq_from
                    self.freq_to=freq_to
                    self.dB=dB
                    self.method=method
                    self.previous_steps = previous_steps

                def run1(self, spectrum):
                    res = []
                    for i in range(1, spectrum.shape[0]):
                        if self.method=='std':
                            res.append(np.std(spectrum[i][(spectrum[0]>=self.freq_from)&(spectrum[0]<self.freq_to)]))
                        if self.method=='mean':
                            res.append(np.mean(spectrum[i][(spectrum[0]>=self.freq_from)&(spectrum[0]<self.freq_to)]))
                        if self.method=='max':
                            res.append(np.max(spectrum[i][(spectrum[0]>=self.freq_from)&(spectrum[0]<self.freq_to)]))

                    return np.max(np.array(res[1:3]))

                def run(self, data):
                    res = np.array([self.run1(d) for d in data[self.previous_steps[0]]])

                    if self.dB==True:
                        res = 20*np.log10(abs(res)/1e-6) #dB로 변환
                    else:
                        res = abs(res)

                    return res.reshape(-1,1)

            data = {"GenerateSpectrum":<spectrum 생성 결과>}
            spec = FeatureSpectrum()
            spec.run(data)

    -------

    Parameters:
    -------
        steps: list
            적용할 전처리 리스트
    -------
    """

    def __init__(self, step_list=None):
        if step_list is not None:
            self.steps = [(step.name, step) for step in step_list]

    def _run1(
        self,
        input_name,
        input_value,
        steps=None,
        feature_names=None,
        verbose=0,
    ):
        if steps is None:
            steps = self.steps
        datum = {input_name: input_value}
        error_indicator = False
        for name, func in steps:
            try:
                datum.update({name: func.run(datum, verbose=verbose)})
            except Exception as e:
                error_indicator = True
                if verbose == 1:
                    print(e)
                    print(f"{input_value} cannot be processed. Error occurs at {name}.")
                break

        if (feature_names is not None) and (error_indicator is False):
            datum = {key: datum[key] for key in feature_names}
            if input_name == "File":
                datum[input_name] = input_value

        return datum

    def run(self, input_name, input_values, feature_names=None, parallel=4, verbose=0):
        """
        run

        Description:
        -------
            데이터에 전처리 과정을 적용하고 결과를 생성
        -------

        Parameters:
        -------
            X: dictionary
                key: 이전 실행 단계
                value: 이전 실행 단계의 결과 데이터
            augment_steps: list
                결과값을 받을 전처리 이름의 리스트
        -------
        Returns:
        -------
            data: dictionary
                key: 전처리 이름
                value: 전처리 결과 데이터
        -------
        """
        N = len(input_values)
        if parallel > 1:
            with Pool(parallel) as p:
                data = list(
                    tqdm(
                        p.imap(
                            _Run1(
                                self.steps,
                                input_name,
                                input_values,
                                feature_names,
                                verbose,
                            ),
                            range(N),
                        ),
                        total=N,
                    )
                )

        else:
            data = [None] * len(input_values)
            if verbose == 0:
                enumerator = enumerate(input_values)
            else:
                enumerator = enumerate(tqdm(input_values))
            for i, input_value in enumerator:
                data[i] = {input_name: input_value}
                for name, func in self.steps:
                    try:
                        data[i].update({name: func.run(data[i], verbose=verbose)})
                    except Exception as e:
                        if verbose == 1:
                            print(e)
                            print(
                                f"{input_value} cannot be processed. Error occurs at {name}."
                            )
                        break

                if feature_names is not None:
                    data[i] = {key: data[i][key] for key in feature_names}
                    if input_name == "File":
                        data[i][input_name] = input_value

        return data

    def make_pipes(self, pipe_descs, _return=False):
        """
        pipeline

        Description:
        -------
            파이프라인 생성을 도와주는 함수
            (func function, previous_step str, step_name str, choose_data str, input_type str, kwargs) 의 형태로 입력
        -------

        Parameters:
        -------
            name: str
                파이프 이름
            previous_step: str
                전 단계의 파이프 이름
            func: function
                파이프에서 실행할 함수
            choose_data: str
                파이프 입력 데이터 종류: both, domain, magitude
            input_type: str
                처리할 입력 데이터의 형태: file, by_test
            __params__: dict
                func 함수에 대한 추가 arguments
        -------
        """

        pipes_list = []
        for desc in pipe_descs:
            if len(desc) < 6:
                add_args = {}
            else:
                add_args = desc[5]
            pipe = Pipe(
                previous_step=desc[1],
                name=desc[2],
                func=desc[0],
                choose_data=desc[3],
                input_type=desc[4],
                add_args=add_args,
            )
            pipes_list.append(pipe)

        self.steps = [(step.name, step) for step in pipes_list]
        self.names = [step.name for step in pipes_list]
        self.params = [step.name for step in pipes_list]

        if _return is True:
            return pipes_list

    def augment_features(
        self,
        prep_data,
        feature_names,
        prefix_names=None,
        file_names=False,
        test_no=[0, 1, 2],
    ):
        """
        augment features

        Description:
        -------
            선택한 feature들을 하나의 numpy array로 연결
        -------

        Parameters:
        -------
            prep_data: dictionary
                Pipeline의 run 으로 생성된 데이터
            prep_data_features: dictionary
                Pipeline의 run 으로 생성된 변수 이름
            augment_steps: list
                결과값을 받을 전처리 이름의 리스트
            aggreate_tests: tuple(length=2)
                반복테스트를 집계할 지 여부. (방법, 집계대상 테스트의 리스트) Ex. ('max', [0,1,2]), ('max', [1,2])

        -------
        Returns:
        -------
            augmented_data: numpy array(shape=[N,m], N=파일 개수, m=파생변수 개수)
                분석 모델에 입력가능한 형태로 변형된 데이터
        -------
        """

        def agg_tests(data):
            return np.concatenate([d for d in data.transpose()])

        def _to_df(d, prefix_names):
            try:
                df = pd.DataFrame(
                    {
                        prefix_names + feature_name: d[feature_name]
                        for feature_name in feature_names
                    }
                )
            except Exception as e:
                print(e)
                df = pd.DataFrame(
                    {
                        prefix_names + feature_name: [None] * len(test_no)
                        for feature_name in feature_names
                    }
                )

            if file_names is True:
                df["File"] = np.repeat(d["File"], len(test_no))

            return df

        if prefix_names is None:
            augmented_data = pd.concat([_to_df(d, "") for d in prep_data]).reset_index(
                drop=True
            )
        else:
            augmented_data = pd.concat(
                [_to_df(d, prefix_names) for d in prep_data]
            ).reset_index(drop=True)

        augmented_data["TEST_NO"] = np.tile(
            [tn + 1 for tn in test_no], int(augmented_data.shape[0] / len(test_no))
        )

        if file_names is True:
            augmented_data["File"] = np.repeat(
                [d["File"] for d in prep_data], len(test_no)
            )

        return augmented_data


class Pipe:
    def __init__(
        self,
        func=None,
        previous_step=None,
        name=None,
        choose_data="both",
        input_type="by_test",
        add_args={},
    ):
        """
        pipeline

        Description:
        -------
            파이프 생성
        -------

        Parameters:
        -------
            name: str
                파이프 이름
            previous_step: str
                전 단계의 파이프 이름
            func: function
                파이프에서 실행할 함수
            choose_data: str
                파이프 입력 데이터 종류: both, domain, magitude
            input_type: str
                처리할 입력 데이터의 형태: file, by_test
            __params__: dict
                func 함수에 대한 추가 arguments
        -------
        """
        self.name = name
        self.previous_step = previous_step
        self.func = func
        self.choose_data = choose_data
        self.input_type = input_type  # type: file: 하나의 파일, by_test: 각 소음테스트별
        self.__params__ = add_args

    ## MC의 경우, 3번의 테스트 결과가 하나의 파일에서 생성되기 때문에
    ## 전처리를 각 테스트 결과별로 수행해야 함
    def run1(self, data):
        """
        하나의 파일을 처리하는 코드
        """
        try:
            num_dim = len(np.array(data, dtype=object).shape)
        except:
            num_dim = 1

        if self.input_type == "by_test":
            if num_dim == 3:
                if self.choose_data == "both" or self.choose_data == "xy":
                    res = [
                        self.func(x=data1[1], y=data1[0], **self.__params__)
                        for data1 in data
                    ]
                elif self.choose_data == "domain" or self.choose_data == "x":
                    res = [self.func(x=data1[1], **self.__params__) for data1 in data]
                elif self.choose_data == "magnitude" or self.choose_data == "y":
                    res = [self.func(y=data1[0], **self.__params__) for data1 in data]
                else:
                    res = None
            else:
                res = [self.func(y=data1, **self.__params__) for data1 in data]
        elif self.input_type == "by_test_two_inputs":
            res = [
                self.func(x=data[0][i], y=data[1][i], **self.__params__)
                for i in range(len(data[0]))
            ]
        else:
            res = None

        try:
            res = np.array(res)
        except:
            res = res

        return res

    def run(self, data, verbose=0):
        """
        여러 파일을 처리하는 코드
        """
        if self.input_type == "file":
            res = self.func(y=data[self.previous_step], **self.__params__)
        elif self.input_type == "by_test":
            res = self.run1(data[self.previous_step])
        elif self.input_type == "by_test_two_inputs":
            res = self.run1((data[self.previous_step[0]], data[self.previous_step[1]]))
        else:
            res = None

        return res
