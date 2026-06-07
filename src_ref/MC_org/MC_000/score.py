# For Clf
import h5py
import pandas as pd
from classify import Classifier
from preprocess_signal_data import Run_
from pkgs.pipelines import FeatureDeployed as Featurizing
from pkgs.utils import read_json

# SENTIMENT
POSITIVE = "POSITIVE"
NEGATIVE = "NEGATIVE"
NEUTRAL = "NEUTRAL"
SENTIMENT_THRESHOLDS = (0.4, 0.7)
SEQUENCE_LENGTH = 300


# Called when the deployed service starts
def init():
    global preproc, loaded_model, model_version, column_names, line, cut_off, rule_model_params, ml_model_params, model_path, error_dir

    model_version = "03_051_01"  # model_name 입력
    file_name = "deployment/data/pkl/{}.pickle".format(model_version)
    edge_config = "deployment/config/edge_config.yml"
    feature_params = read_json("src/config/feature051_parameters.json")
    column_names = read_json("src/config/model_03_051_01_features.json")[
        "feature_names"
    ]

    # load line info
    with open(edge_config, "r") as stream:
        try:
            edge_config = yaml.load(stream, Loader=yaml.BaseLoader)
            line = edge_config["config"]["line"]["name"]
            cut_off = float(edge_config["config"]["param"]["cutoff"])
            rule_model_params = edge_config["config"]["param"]["rule_model"]
            ml_model_params = edge_config["config"]["param"]["ml_model"]

        except yaml.YAMLError as exc:
            print("line config error: ", exc)

    # set preprocess class
    for param_, dic_ in ml_model_params.items():
        for key_, value_ in dic_.items():
            feature_params[param_][key_] = float(value_)
    preproc = init_preprocessor(
        feature_params=feature_params, Featurizing=Featurizing, test_no=[0, 1, 2]
    )

    # load model
    model_path = Model.get_model_path(file_name)
    with open(model_path, "rb") as f:
        loaded_model = pickle.load(f)

    error_dir = "/home/data/error_file/"
    try:
        os.makedirs(error_dir, mode=777)
    except:
        pass


# Handle requests to the service
def run(data):
    try:
        data_steps = preprocess(
            preproc=preproc,
            input_values=[data],
            feature_names=None,
            input_name="ReadData",
            parallel=1,
            verbose=0,
        )

        df = classify(
            data_steps=data_steps,
            feature_names=preproc.featurizer.params["feature_names"],
            column_names=column_names,
            cut_off=cut_off,
            rule_model_params=rule_model_params,
            mode="or",
        )

        return df
    except Exception as e:
        error = str(e)
        return error


def init_preprocessor(feature_params, Featurizing, test_no, input_type="ReadData"):
    # set preprocess class
    preproc = Run_(
        feature_params=feature_params, Featurizing=Featurizing, test_no=test_no
    )
    if input_type == "ReadData":
        preproc.pl.steps = [
            (step_name, step)
            for step_name, step in preproc.pl.steps
            if step.name != "ReadData"
        ]

    return preproc


def preprocess(
    preproc,
    input_values,
    feature_names,
    input_name="ReadData",
    parallel=1,
    verbose=0,
):
    # Preprocess
    data_steps = preproc.pl.run(
        input_name=input_name,
        input_values=[input_values],
        feature_names=feature_names,
        parallel=parallel,
        verbose=verbose,
    )

    return data_steps


def classify(data_steps, feature_names, column_names, cut_off, rule_model_params, mode):
    # Preprocess
    df = pd.DataFrame({name: data_steps[0][name] for name in feature_names})
    df["TEST_NO"] = [0, 1, 2]
    df_features = df[column_names]

    # Predict
    df["PROB"] = loaded_model.predict_proba(df_features)[:, 1]

    # Classify
    df["ML_R"] = df.PROB >= cut_off
    df["ML_R"] = df.ML_R.astype(int)
    df = Classifier.classify_all(
        df=df[df.TEST_NO.isin(preproc.featurizer.params["test_no"])],
        rule_cutoffs=rule_model_params,
        mode=mode,
    )
    return df
