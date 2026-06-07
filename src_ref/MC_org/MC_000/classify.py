# read preprocessed data
# structure data so that it can used for modeling
#  - merge test results
# select columns for ml model
import os
import argparse
import pathlib
import numpy as np
import pandas as pd
from pkgs.utils import read_json
from azureml.core import Run


class Classifier:
    @classmethod
    def ml_classify(cls, df, mode="or"):
        df = df.copy()
        index_ng = df.ML_R == 1
        ng_test_no = "/".join(df[index_ng].TEST_NO.astype(str).tolist())
        if mode == "or":
            p = df.PROB.max()
            r = df.ML_R.max()
        elif mode == "and":
            p = df.PROB.min()
            r = df.ML_R.prod()

        return pd.DataFrame({"ML_R": [r], "PROB": [p], "ML_NG_TEST_NO": [ng_test_no]})

    @classmethod
    def rule_classify(cls, df, cutoff=None):
        """
        규칙 기반 분류 수행
        """
        df = df.copy()
        # 1. WindowedSTD
        df["FTUR_WVFM_STDDEV_R"] = (
            df["FTUR_WVFM_STDDEV"] > float(cutoff["FTUR_WVFM_STDDEV"])
        ).astype(int)
        # 2. Energy
        df["FTUR_ENRG_R"] = (df["FTUR_ENRG"] > float(cutoff["FTUR_ENRG"])).astype(int)

        # 3. Classify
        df["RULE_R"] = df[["FTUR_WVFM_STDDEV_R", "FTUR_ENRG_R"]].max(axis=1)

        index_ng = df.RULE_R == 1
        ng_test_no = "/".join(df[index_ng].TEST_NO.astype(str).tolist())
        return pd.DataFrame(
            {"RULE_R": [df.RULE_R.max()], "RULE_NG_TEST_NO": [ng_test_no]}
        )

    @classmethod
    def check_test(cls, df, cutoff=None):
        """
        테스트 불량 탐지 수행
        """
        # 1. Trigger
        df["FTUR_TRGER_R"] = (df["FTUR_TRGER"] < float(cutoff["FTUR_TRGER"])).astype(
            int
        )

        # # 2. Polynomial
        # def mahalanobis(x=None, mean=None, cov=None):
        #     tmp1 = np.matmul(x-mean, linalg.inv(cov))
        #     res = np.array([np.sum(tmp1[i]*(x-mean)[i]) for i in range(x.shape[0])])
        #     return res

        # m_dist = np.sqrt(mahalanobis(x=data_dict['FeaturePolyCoef'], mean=rule_params['FeaturePolyCoef']['mean'], cov=rule_params['FeaturePolyCoef']['cov']))

        # res2 = pd.Series((m_dist>rule_params['FeaturePolyCoef']['cut_off'])).astype(int).values

        # return pd.DataFrame(np.array([res1, res2]).transpose(), columns=['FeatureTriggers','FeaturePolyCoef'])
        # 3. Classify
        # df["TEST_NG_R"] = df[["FTUR_ENRG_R"]].max(axis=1)
        df["TEST_NG_R"] = df.FTUR_TRGER_R

        index_ng = df.TEST_NG_R == 1
        ng_test_no = "/".join(df[index_ng].TEST_NO.astype(str).tolist())
        return pd.DataFrame(
            {"TEST_NG_R": [df.TEST_NG_R.max()], "TEST_NG_TEST_NO": [ng_test_no]}
        )

    @classmethod
    def classify_all(cls, df, rule_cutoffs, mode="or"):
        df = df.copy()
        df_ml = cls.ml_classify(df, mode=mode)
        df_rule = cls.rule_classify(df, cutoff=rule_cutoffs)
        df_test = cls.check_test(df, cutoff=rule_cutoffs)
        df_all = pd.concat([df_ml, df_rule, df_test], axis=1)
        df_all["R"] = np.where(
            df_all.TEST_NG_R == 1, 2, np.where(df_all.RULE_R == 1, 1, df_all.ML_R)
        )

        return df_all

    @classmethod
    def classify_batch(cls, df, rule_cutoffs, label="", mode="or"):
        if label in df.columns:
            df = (
                df.groupby(["FPATH", label])
                .apply(cls.classify_all, rule_cutoffs=rule_cutoffs, mode=mode)
                .reset_index()
            )
        else:
            df = (
                df.groupby("FPATH")
                .apply(cls.classify_all, rule_cutoffs=rule_cutoffs, mode=mode)
                .reset_index()
            )
        return df


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
    label="SR_RELABELED",
    classifier_mode="or",
    ml_cutoffs={1: 0.3, 2: 0.3, 3: 0.4, 4: 0.3, 5: 0.3, 6: 0.3, 7: 0.3, 8: 0.3},
    rule_cutoffs={"FTUR_ENRG": 0.00003, "FTUR_WVFM_STDDEV": 0.12, "FTUR_TRGER": 1},
    output_upload_dir="<NO_UPLOAD>",
):

    # Read data
    df = pd.read_csv(input_data)
    print("Original Shape: ", end="")
    print(df.shape)

    # Classifiy
    df_cutoff = pd.DataFrame(ml_cutoffs, index=[0]).melt()
    df_cutoff.columns = ["LID", "CUTOFF"]
    df = df.merge(df_cutoff, on="LID")
    df["ML_R"] = df.PROB >= df.CUTOFF
    df["ML_R"] = df.ML_R.astype(int)

    df_classified = Classifier.classify_batch(
        df=df, rule_cutoffs=rule_cutoffs, label=label, mode=classifier_mode
    )
    # Save data
    os.makedirs(os.path.dirname(output_data), exist_ok=True)
    df_classified.to_csv(output_data, index=False)

    # upload data
    if output_upload_dir != "<NO_UPLOAD>":
        upload_data(output_data, output_upload_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--input_data")
    parser.add_argument("--output_data")
    parser.add_argument("--classifier_mode", default="or")
    parser.add_argument("--label", default="SR_RELABELED")
    parser.add_argument("--cfg_cutoff_json", default="config/cutoffs.json")
    parser.add_argument("--output_upload_dir", default="<NO_UPLOAD>")
    args = parser.parse_args()

    args.cutoffs = read_json(args.cfg_cutoff_json)
    args.ml_cutoffs = {
        int(key): value for key, value in args.cutoffs["ml_cutoff"].items()
    }
    args.rule_cutoffs = args.cutoffs["rule_cutoff"]

    main(
        input_data=args.input_data,
        output_data=args.output_data,
        label=args.label,
        classifier_mode=args.classifier_mode,
        ml_cutoffs=args.ml_cutoffs,
        rule_cutoffs=args.rule_cutoffs,
        output_upload_dir=args.output_upload_dir,
    )
