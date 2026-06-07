import os
import datetime
import argparse
import pathlib
from pkgs.utils import read_json
from pkgs.pipelines import Feature051 as Featurizing
from pkgs.mc_noisetest.function import preprocess, signal, feature
from azureml.core import Run
import pandas as pd

# csv 파일에 저장
# DB에 저장
# 하나의 쿼리만 수행
# 하나의 쿼리를 나눠서 수행


class Run_:
    def __init__(self, feature_params, Featurizing, test_no=None, lid=None):
        if lid is not None:
            feature_params["FilterActiveNoiseCanceling"][
                "prop_decrease"
            ] = feature_params["params_line"]["FilterActiveNoiseCanceling"][
                "prop_decrease"
            ][
                str(lid)
            ]
        self.featurizer = Featurizing(params=feature_params)
        if test_no is None:
            test_no = self.featurizer.params["test_no"]
        self.pr = preprocess(test_no=test_no)
        self.sg = signal()
        self.ft = feature()
        self.pl = self.featurizer.make_pipeline(self.pr, self.sg, self.ft)

    def run_test(
        self,
        data,
        feature_names,
        parallel,
        input_values,
        input_name="File",
        verbose=0,
    ):
        data_steps = self.pl.run(
            input_name=input_name,
            input_values=input_values,
            feature_names=feature_names,
            parallel=parallel,
            verbose=verbose,
        )
        # df = self.featurizer.make_feature(
        #     pl,
        #     data_steps,
        #     feature_names,
        #     data=data,
        #     test_no=self.featurizer.params["test_no"],
        # )

        return data_steps

    def run(
        self,
        data,
        feature_names,
        parallel,
        input_values,
        input_name="File",
        lid=None,
        verbose=0,
    ):
        data_steps = self.pl.run(
            input_name=input_name,
            input_values=input_values,
            feature_names=feature_names,
            parallel=parallel,
            verbose=verbose,
        )
        df = self.featurizer.make_feature(
            self.pl,
            data_steps,
            feature_names,
            data=data,
            test_no=self.featurizer.params["test_no"],
        )

        return df


class _CheckRunTime:
    @staticmethod
    def check_start(message="", verbose=1):
        time_start = datetime.datetime.now()

        if verbose == 1:
            print(f"{str(time_start)[11:19]} | {message}")
        return time_start

    @staticmethod
    def check_end(time_start, message="", verbose=1):
        # # 수행시간 확인
        time_end = datetime.datetime.now()
        duration_in_s = (time_end - time_start).total_seconds()
        min_sec = divmod(duration_in_s, 60)

        if verbose == 1:
            print(
                f"{str(time_end)[11:19]} | {str(min_sec[0])}m {str(min_sec[1])}s | {message}"
            )


def upload_data(output_data, output_upload_dir):
    # upload data
    # get file name and dir absolute
    file_path = str(pathlib.Path(output_data).absolute())
    rel_root = os.path.dirname(file_path)
    # upload
    run = Run.get_context()
    ws = run.experiment.workspace
    datastore = ws.get_default_datastore()
    datastore.upload_files(
        files=[file_path],
        relative_root=rel_root,
        target_path=output_upload_dir,
        overwrite=True,
    )
    print(f"{output_data} uploaded to {datastore.name}/{output_upload_dir}")


def main(
    input_data,
    output_data,
    output_path=None,
    input_rawdata_dir=None,
    run_iter_list=["LID", "DT"],
    run_num_parallel=1,
    feature_params=None,
    verbose=1,
    output_upload_dir="<NO_UPLOAD>",
):
    if input_rawdata_dir is not None:
        os.symlink(input_rawdata_dir, "/mc-origin-data-jyt", target_is_directory=True)
    print("Start preprocessing...")
    # Run Featurization and Save the Result
    # Read Data
    data = pd.read_csv(input_data)
    if run_iter_list is None:
        run = Run_(feature_params=feature_params, Featurizing=Featurizing)
        # Run
        time_start = _CheckRunTime.check_start("Start preprocessing...")
        df = run.run(
            data=data,
            input_values=data.FPATH.values,
            feature_names=run.featurizer.params["feature_names"],
            parallel=run_num_parallel,
            verbose=verbose,
        )
        # Save Data
        df.to_csv(output_data, index=False)
        # upload data
        if output_upload_dir != "<NO_UPLOAD>":
            upload_data(output_data, output_upload_dir)
        _CheckRunTime.check_end(time_start, "Done.")
    elif run_iter_list is not None:
        # Set indices for iteration
        df_list = []
        iter_list = data[run_iter_list].drop_duplicates().sort_values(run_iter_list)
        # run and save staging files iteratevely
        for i in iter_list.index:
            lid = iter_list.LID[i]
            dt = iter_list.DT[i]

            run = Run_(feature_params=feature_params, Featurizing=Featurizing, lid=lid)

            time_start = _CheckRunTime.check_start(
                f"Start preprocessing : {str(lid)} line / {str(dt)} ..."
            )

            # Read Data
            tmp_data = data[(data.LID == lid) & (data.DT == dt)]
            if tmp_data[["FPATH"]].dropna().shape[0] == 0:
                _CheckRunTime.check_end(time_start, "Not enough data.")
                continue

            # Run
            df = run.run(
                data=tmp_data,
                input_values=tmp_data.FPATH.values,
                feature_names=run.featurizer.params["feature_names"],
                parallel=run_num_parallel,
                verbose=verbose,
            )
            # df_list.append(df)
            _CheckRunTime.check_end(time_start, "Done.")

            ###
            if output_path is not None:
                output_file_path = os.path.join(output_path, output_data)
            else:
                output_file_path = output_data
            os.makedirs(os.path.dirname(output_file_path), exist_ok=True)
            df.to_csv(f"{output_file_path}_{lid}_{dt}.csv", index=False)

            # upload data
            if output_upload_dir != "<NO_UPLOAD>":
                upload_data(f"{output_data}_{lid}_{dt}.csv", output_upload_dir)

        # # Save
        # df = pd.concat(df_list)
        # df.to_csv(output_data, index=False)
        # # upload data
        # if output_upload_dir != "<NO_UPLOAD>":
        #     upload_data(output_data, output_upload_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # input, output arguments
    parser.add_argument("--input_data")
    parser.add_argument("--output_data")
    parser.add_argument("--output_path")
    parser.add_argument("--input_rawdata_dir", default=None)
    # run arguments
    parser.add_argument("--run_iter", dest="run_iter", action="store_true")
    parser.add_argument("--not_run_iter", dest="run_iter", action="store_false")
    parser.set_defaults(run_iter=True)
    parser.add_argument("--run_iter_list", nargs="+", default=["DT", "LID"])
    parser.add_argument("--run_num_parallel", type=int, default=1)
    # else
    parser.add_argument(
        "--feature_config",
        default="config/feature01_parameters.json",
    )
    parser.add_argument("--output_upload_dir", default="<NO_UPLOAD>")

    args = parser.parse_args()

    args.feature_params = read_json(args.feature_config)

    if args.run_iter is True:
        args.run_iter_list = args.run_iter_list
    else:
        args.run_iter_list = None

    main(
        input_data=args.input_data,
        output_data=args.output_data,
        output_path=args.output_path,
        input_rawdata_dir=args.input_rawdata_dir,
        run_iter_list=args.run_iter_list,
        run_num_parallel=args.run_num_parallel,
        feature_params=args.feature_params,
        output_upload_dir=args.output_upload_dir,
    )
