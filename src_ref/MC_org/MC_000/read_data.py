import os
import pandas as pd
import pathlib
import argparse
from pkgs.utils import under_sampling, connect_to_mysql
from azureml.core import Run


# csv 파일에 저장
# DB에 저장
# 하나의 쿼리만 수행
# 하나의 쿼리를 나눠서 수행

# input
# DB 접속정보
# Blob 접속정보
# 입력 쿼리 + alpha

# Copyright (c) Microsoft. All rights reserved.
# Licensed under the MIT license.


def _read_query(query=None, under_sampling_param=None):
    try:
        con = connect_to_mysql(database_name="mc_dev")
        data = pd.read_sql(con=con, sql=query)
        con.close()
    except Exception as e:
        raise (e)

    if under_sampling_param is not None:
        weight = {
            under_sampling_param["values"][i]: under_sampling_param["weights"][i]
            for i in range(len(under_sampling_param["values"]))
        }
        data = under_sampling(
            df=data,
            colname=under_sampling_param["colname"],
            weight=weight,
        )

    return data


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
    input_query,
    output_data,
    under_sampling_param={
        "colname": "R",
        "values": [0, 1],
        "weights": [0.05, 1.0],
    },
    output_upload_dir="<NO_UPLOAD>",
):
    # read data
    data = _read_query(query=input_query, under_sampling_param=under_sampling_param)

    # save data
    os.makedirs(os.path.dirname(output_data), exist_ok=True)
    data.to_csv(output_data, index=False)

    # upload data
    if output_upload_dir != "<NO_UPLOAD>":
        upload_data(output_data, output_upload_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # input arguments
    parser.add_argument(
        "--input_query", default="SELECT * FROM analysis_v3_label WHERE DT='2020-02-08'"
    )
    # under sampling arguments
    parser.add_argument("--under_sampling", dest="u_spling", action="store_true")
    parser.add_argument("--no_under_sampling", dest="u_spling", action="store_false")
    parser.set_defaults(u_spling=True)
    parser.add_argument("--under_sampling_colname", default="R")
    parser.add_argument("--under_sampling_values", nargs="+", type=int, default=[0, 1])
    parser.add_argument(
        "--under_sampling_weights", nargs="+", type=float, default=[0.05, 1.0]
    )
    # output arguments
    parser.add_argument("--output_data", default="outputs/output.csv")
    # upload arguments
    parser.add_argument("--output_upload_dir", default="<NO_UPLOAD>")

    args = parser.parse_args()
    if args.u_spling is True:
        args.under_sampling_param = {
            "colname": args.under_sampling_colname,
            "values": args.under_sampling_values,
            "weights": args.under_sampling_weights,
        }
    else:
        args.under_sampling_param = None

    main(
        input_query=args.input_query,
        under_sampling_param=args.under_sampling_param,
        output_data=args.output_data,
        output_upload_dir=args.output_upload_dir,
    )
