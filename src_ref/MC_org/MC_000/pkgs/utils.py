# -*- coding: utf-8 -*-
import mysql.connector
import pandas as pd
import numpy as np
import glob
import re
import datetime
import h5py
import json
from sqlalchemy import create_engine
from random import choice
from string import ascii_lowercase


def read_json(cfg_path):
    with open(cfg_path, "r") as st_json:
        loaded = json.load(st_json)
    return loaded


def _catch(func, handle=lambda e: e, pipe_name=None, *args, **kwargs):
    try:
        return func(*args, **kwargs)
    except Exception as e:
        # logging.debug("{e} \n error index is {error_idx}")
        print(f"{e} \n Error occurs at  {pipe_name}")
        #         logging.debug("{} \n error index is {}".format(e, error_idx))
        return None


def random_string_generator(strlen=4):
    """일정 길이의 랜덤한 string을 생성한다."""
    lis = list(ascii_lowercase)
    return "".join(choice(lis) for _ in range(strlen))


def under_sampling(df, colname, weight, drop_unknown_weight=True):
    """colname 컬럼 값 따라 다른 비율로(weight에 정의됨) df를 샘플링한다."""
    if drop_unknown_weight is True:
        df = df[df[colname].isin([key for key in weight.keys()])]

    tmp_list = []
    for value in df[colname].unique():
        tmp = df[df[colname] == value]
        tmp_list.append(tmp.sample(frac=weight[value]))

    return pd.concat(tmp_list).sample(frac=1).reset_index(drop=True)


# mysql 연결함수 정의
def ReadData(y, header="infer", names=None, path="", data_format="Raw"):
    filename = y
    if data_format == "csv":
        data = pd.read_csv(filename, names=["x1", "x2", "x3"]).values
    else:
        with h5py.File(filename) as f:
            data = f["Raw"][:]

    return data.transpose()


def SaveDataBlob(
    local_file_name,
    blob_name,
    container_name="workplace",
    con_string="xxxx",
):
    # Create a blob client using the local file name as the name for the blob
    local_file_name = "analysis_v3_200108-200128.csv"
    blob_name = "FTUR.csv"

    blob_service_client = BlobServiceClient.from_connection_string(con_string)
    blob_client = blob_service_client.get_blob_client(
        container=container_name, blob=blob_name
    )

    # Upload the created file
    with open(local_file_name, "rb") as data:
        blob_client.upload_blob(data)


def connect_to_mysql_engine(
    server_name="mc-mysql",
    database_name="mc",
    user_id="lsisuser@mc-mysql",
    user_password="Tbvjaosvkdnj00",
    endpoint="mysql.database.azure.com",
):
    engine = create_engine(
        "mysql+mysqlconnector://",
        creator=lambda: mysql.connector.connect(
            user=user_id,
            password=user_password,
            host=f"{server_name}.{endpoint}",
            database=database_name,
        ),
    )

    return engine


def connect_to_mysql(
    server_name="mc-mysql",
    database_name="mc",
    user_id="lsisuser@mc-mysql",
    user_password="Tbvjaosvkdnj00",
    endpoint="mysql.database.azure.com",
):
    engine = create_engine(
        "mysql+mysqlconnector://",
        creator=lambda: mysql.connector.connect(
            user=user_id,
            password=user_password,
            host=f"{server_name}.{endpoint}",
            database=database_name,
        ),
    )
    conn = engine.connect()

    return conn


def connect_to_mariadb(
    host="165.244.138.57",
    database_name="mc_edge",
    user_id="lsisuser",
    user_password="Tbvjaosvkdnj00",
    port="3306",
):
    engine = create_engine(
        "mysql+mysqlconnector://",
        creator=lambda: mysql.connector.connect(
            user=user_id,
            password=user_password,
            host=f"{host}",
            database=database_name,
            port=port,
        ),
    )
    conn = engine.connect()

    return conn


class relabel:
    def __init__(self, relabeled_path="/usrData1/relabel_data/", relabeled_list=None):
        self.relabeled_path = relabeled_path
        if relabeled_list is None:
            relabeled_list = (glob.glob(self.relabeled_path + r"*.[csv|CSV]"),)
        self.relabeled_condition = ""
        ## 전수검사 기간
        self.labeled_all = (
            "(LID=1 and DT>='2019-11-12' and DT<='2019-12-06') OR "
            "(LID=2 and DT>='2019-10-26' and DT<='2019-11-29') OR "
            "(LID=3 and DT>='2019-10-22' and DT<='2019-11-29') OR "
            "(LID=4 and DT>='2019-11-12' and DT<='2019-12-06') OR "
            "(LID=5 and DT>='2019-11-12' and DT<='2019-12-06') OR "
            "(LID=6 and DT>='2019-11-18' and DT<='2019-12-30') OR "
            "(LID=7 and DT>='2019-11-18' and DT<='2019-12-10') OR "
            "(LID=8 and DT>='2019-11-18' and DT<='2019-12-10') "
        )

    def _select(
        self, con_engine, from_dt, to_dt, line, cond="", save=True, if_exists="fail"
    ):

        ## parse conditions for naming
        from_dt_name = re.sub("-", "", from_dt)[2:]
        to_dt_name = re.sub("-", "", to_dt)[2:]
        line_name = "_".join(re.findall(r"\d", str(line)))
        filename = (
            "/usrData1/relabel_data/before_relabel/before_jyt_relabel_list_from_"
            + from_dt_name
            + "_to_"
            + to_dt_name
            + "_lid"
            + line_name
            + ".csv"
        )
        dbname = (
            "tmp_relabel_ing_" + from_dt_name + "_to_" + to_dt_name + "_lid" + line_name
        )

        ## parse query conditions
        dt_cond = "(DT>='" + from_dt + "' and DT<='" + to_dt + "')"
        tmp = re.sub(r"\]", r")", str(line))
        tmp = re.sub(r"\[", "(LID = ", tmp)
        line_cond = re.sub(",", " OR LID =", tmp)
        if cond != "":
            cond = " AND " + cond

        ## read data from DB
        con = con_engine.connect()
        tmp_report = pd.read_sql(
            con=con,
            sql="select * from mc.edge_report where "
            + dt_cond
            + " AND "
            + line_cond
            + cond,
        )
        tmp_path = pd.read_sql(
            con=con,
            sql="select * from (select *, concat(VPATH, FNAME) as FPATH, substring(DTFULL, 1,10) as DT from mc.file_path_jyt) A where "
            + dt_cond,
        )
        tmp_human = pd.read_sql(
            con=con,
            sql="select * from mc.sensory_test  where " + dt_cond + " AND " + line_cond,
        )
        tmp_report.DT = tmp_report.DT.astype(str)
        tmp_human.DT = tmp_human.DT.astype(str)
        con.close()

        raw_relabeled_b = (
            tmp_report.set_index(["LID", "BC"])
            .join(tmp_human.loc[:, ["BC", "SR", "LID"]].set_index(["LID", "BC"]))
            .reset_index()
        )
        raw_relabeled_b = (
            raw_relabeled_b.set_index(["BC", "DTFULL"])
            .join(tmp_path[["BC", "DTFULL", "FPATH"]].set_index(["BC", "DTFULL"]))
            .reset_index()
        )
        raw_relabeled_b = raw_relabeled_b.loc[
            :, ["FPATH", "DTFULL", "DT", "LID", "SR", "PROB", "R", "CUTOFF", "V"]
        ]
        raw_relabeled_b["RELABEL_METHOD"] = np.where(
            (raw_relabeled_b.R == 1) | (-raw_relabeled_b.SR.isna()), "MANUAL", "RULE"
        )
        raw_relabeled_b["SR_RELABELED"] = None
        raw_relabeled_b["TEST_NG"] = None

        if save is True:
            raw_relabeled_b.to_csv(filename, encoding="CP949", index=False)
            df_relabel_reviewed = raw_relabeled_b.copy()
            df_relabel_reviewed["UPDATE_DT"] = datetime.datetime.now()
            df_relabel_reviewed.UPDATE_DT.astype("datetime64[s]")
            df_relabel_reviewed["TEST_NG"] = None
            df_relabel_reviewed.to_sql(
                con=con_engine, name=dbname, index=False, if_exists=if_exists
            )

        return raw_relabeled_b, filename, dbname

    def merge_tables(self, data_list, data_review_list=None):
        df_relabeled = pd.concat(data_list).reset_index(drop=True)
        df_relabeled["FNAME"] = df_relabeled.FPATH.str.extract(r"([^/]+$)")
        df_relabeled["VPATH"] = df_relabeled.FPATH.str.replace(r"([^/]+$)", "")
        df_relabeled["BC"] = df_relabeled.FNAME.str.extract(r"([^_]+_[^_]+)_")
        df_relabeled["DT"] = df_relabeled.FNAME.str.extract(r"(\d{4}-\d{2}-\d{2})")
        df_relabeled["HS"] = df_relabeled.FNAME.str.extract(
            r"\d{4}-\d{2}-\d{2}-(\d{2}-\d{2}-\d{2})"
        )[0].str.replace("-", ":")
        df_relabeled["DTFULL"] = (df_relabeled["DT"] + " " + df_relabeled["HS"]).astype(
            "datetime64[ns]"
        )

        df_relabeled = df_relabeled.loc[
            :,
            [
                "FPATH",
                "BC",
                "DTFULL",
                "VPATH",
                "FNAME",
                "LID",
                "SR",
                "SR_RELABELED",
                "RELABEL_METHOD",
                "TEST_NG",
            ],
        ].reset_index(drop=True)

        ## 재검토 결과 반영
        # 마지막 재검토 결과가 반영되도록, 시간 순서대로 앞의 재검토 결과부터 덮어쓰기.
        if data_review_list is not None:
            df_relabeled_fin = df_relabeled.copy()
            for data_review in data_review_list:
                data_review["FNAME"] = data_review.FPATH.str.extract(r"([^/]+$)")
                data_review = data_review.loc[:, ["FNAME", "SR_RELABELED"]].rename(
                    columns={"SR_RELABELED": "SR_RELABELED2"}
                )
                df_relabeled_fin = (
                    df_relabeled_fin.set_index("FNAME")
                    .join(data_review.set_index("FNAME"))
                    .reset_index()
                )
                df_relabeled_fin["SR_RELABELED"] = np.where(
                    df_relabeled_fin.SR_RELABELED2.isna(),
                    df_relabeled_fin.SR_RELABELED,
                    df_relabeled_fin.SR_RELABELED2,
                )
                df_relabeled_fin = df_relabeled_fin.drop("SR_RELABELED2", axis=1)
        else:
            df_relabeled_fin = df_relabeled

        return df_relabeled_fin

        # edge_report
        # file_path
        # LID condition
        # DT condition
        # etc condition

    def read_relabeled_csv(self):
        """
        재라벨 완료된 파일을 읽어 리스트로 반환
        - 메카시스 불량데이터에 대해 재라벨링 수행
        - 관능검사 결과가 있거나 모델 예측 결과가 불량인 경우
        """
        #### 1. 2019-10-22 ~ 2019-10-25 Line4 재라벨링
        tmp_relabeled = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191022_to_191025_lid3.csv",
            encoding="CP949",
        )
        tmp_relabeled["RELABEL_METHOD"] = np.where(
            (tmp_relabeled.R == 1) | np.logical_not(tmp_relabeled.SR.isna()),
            "MANUAL",
            "RULE",
        )
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled1 = tmp_relabeled.copy()
        df_relabeled1.shape

        #### 2. 2019-10-28 ~ 2019-10-31 Line3,4 재라벨링
        tmp_relabeled = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191026_to_191031_lid2_3.csv",
            encoding="CP949",
        )
        tmp_relabeled["RELABEL_METHOD"] = np.where(
            (tmp_relabeled.R == 1) | np.logical_not(tmp_relabeled.SR.isna()),
            "MANUAL",
            "RULE",
        )
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled2 = tmp_relabeled.copy()
        df_relabeled2.shape

        #### 3. 2019-11-01 ~ 2019-11-08 Line3,4 재라벨링
        tmp_relabeled = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191101_to_191108_lid2_3.csv",
            encoding="CP949",
        )
        tmp_relabeled["RELABEL_METHOD"] = np.where(
            (tmp_relabeled.R == 1) | np.logical_not(tmp_relabeled.SR.isna()),
            "MANUAL",
            "RULE",
        )
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled3 = tmp_relabeled.copy()
        df_relabeled3.shape

        #### 4. 2019-11-12 ~ 2019-11-15 LID12345 재라벨링 대상 (2019-11-11 은 휴일)
        tmp_relabeled = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191111_to_191115_lid1_2_3_4_5.csv",
            encoding="CP949",
        )
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled4 = tmp_relabeled.copy()
        df_relabeled4.shape

        #### 5. 2019-11-18 ~ 2019-11-22 LID12345678 재라벨링 대상
        ## DB결과를 csv 파일로 저장
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled_manual = pd.read_sql(con=con, sql='select * from tmp_relabel_ing_191118_to_191122_lid1_2_3_4_5_6_7_8')
        # con.close()
        # tmp_relabeled_manual.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191118_to_191122_lid1_2_3_4_5_6_7_8.csv', index=False)

        tmp_relabeled_manual = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191118_to_191122_lid1_2_3_4_5_6_7_8.csv"
        )
        tmp_relabeled_b = pd.read_csv(
            "/usrData1/relabel_data/before_relabel/before_jyt_relabel_list_from_191118_to_191122_lid1_2_3_4_5_6_7_8.csv"
        )
        tmp_relabeled = (
            tmp_relabeled_b.set_index("FPATH")
            .join(
                tmp_relabeled_manual[["FPATH", "SR_RELABELED", "TEST_NG"]]
                .rename(
                    columns={"SR_RELABELED": "SR_RELABELED2", "TEST_NG": "TEST_NG2"}
                )
                .set_index("FPATH")
            )
            .reset_index()
        )
        tmp_relabeled["SR_RELABELED"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.SR_RELABELED2,
            tmp_relabeled.SR_RELABELED,
        )
        tmp_relabeled["TEST_NG"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.TEST_NG2,
            tmp_relabeled.TEST_NG,
        )
        tmp_relabeled = tmp_relabeled.drop(["SR_RELABELED2", "TEST_NG2"], axis=1)
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled5 = tmp_relabeled.copy()
        df_relabeled5.shape

        #### 6. 2019-11-25 ~ 2019-11-29 LID12345678 재라벨링 대상
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled_manual = pd.read_sql(con=con, sql='select * from tmp_relabel_ing_191125_to_191129_lid1_2_3_4_5_6_7_8')
        # con.close()
        # tmp_relabeled_manual.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191125_to_191129_lid1_2_3_4_5_6_7_8.csv', index=False)

        tmp_relabeled_manual = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191125_to_191129_lid1_2_3_4_5_6_7_8.csv"
        )
        ################임시
        tmp_relabeled_manual["SR_RELABELED"] = np.where(
            -tmp_relabeled_manual.SR_RELABELED.isna(),
            tmp_relabeled_manual.SR_RELABELED,
            np.where(
                -tmp_relabeled_manual.SR.isna(),
                tmp_relabeled_manual.SR,
                tmp_relabeled_manual.R,
            ),
        )
        ##################
        tmp_relabeled_b = pd.read_csv(
            "/usrData1/relabel_data/before_relabel/before_jyt_relabel_list_from_191125_to_191129_lid1_2_3_4_5_6_7_8.csv"
        )
        tmp_relabeled = (
            tmp_relabeled_b.set_index("FPATH")
            .join(
                tmp_relabeled_manual[["FPATH", "SR_RELABELED", "TEST_NG"]]
                .rename(
                    columns={"SR_RELABELED": "SR_RELABELED2", "TEST_NG": "TEST_NG2"}
                )
                .set_index("FPATH")
            )
            .reset_index()
        )
        tmp_relabeled["SR_RELABELED"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.SR_RELABELED2,
            tmp_relabeled.SR_RELABELED,
        )
        tmp_relabeled["TEST_NG"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.TEST_NG2,
            tmp_relabeled.TEST_NG,
        )
        tmp_relabeled = tmp_relabeled.drop(["SR_RELABELED2", "TEST_NG2"], axis=1)
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled6 = tmp_relabeled.copy()
        df_relabeled6.shape

        #### 7. 2019-12-02 ~ 2019-12-06 LID12345678 재라벨링 대상
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled_manual = pd.read_sql(con=con, sql='select * from tmp_relabel_ing_191202_to_191206_lid1_2_3_4_5_6_7_8')
        # con.close()
        # tmp_relabeled_manual.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191202_to_191206_lid1_2_3_4_5_6_7_8.csv', index=False)

        tmp_relabeled_manual = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191202_to_191206_lid1_2_3_4_5_6_7_8.csv"
        )
        tmp_relabeled_b = pd.read_csv(
            "/usrData1/relabel_data/before_relabel/before_jyt_relabel_list_from_191202_to_191206_lid1_2_3_4_5_6_7_8.csv"
        )
        tmp_relabeled = (
            tmp_relabeled_b.set_index("FPATH")
            .join(
                tmp_relabeled_manual[["FPATH", "SR_RELABELED", "TEST_NG"]]
                .rename(
                    columns={"SR_RELABELED": "SR_RELABELED2", "TEST_NG": "TEST_NG2"}
                )
                .set_index("FPATH")
            )
            .reset_index()
        )
        tmp_relabeled["SR_RELABELED"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.SR_RELABELED2,
            tmp_relabeled.SR_RELABELED,
        )
        tmp_relabeled["TEST_NG"] = np.where(
            tmp_relabeled.RELABEL_METHOD == "MANUAL",
            tmp_relabeled.TEST_NG2,
            tmp_relabeled.TEST_NG,
        )
        tmp_relabeled = tmp_relabeled.drop(["SR_RELABELED2", "TEST_NG2"], axis=1)
        tmp_relabeled.SR = tmp_relabeled.SR.fillna(0)
        tmp_relabeled.SR_RELABELED = np.where(
            tmp_relabeled.SR_RELABELED.isna(),
            tmp_relabeled.SR,
            tmp_relabeled.SR_RELABELED,
        )
        df_relabeled7 = tmp_relabeled.copy()
        df_relabeled7.shape

        return [
            df_relabeled1,
            df_relabeled2,
            df_relabeled3,
            df_relabeled4,
            df_relabeled5,
            df_relabeled6,
            df_relabeled7,
        ]

    def read_relabeled_review_csv(self):
        """
        재라벨 완료 후 재확인된 파일을 읽어 리스트로 반환
        """
        #### 1. 2019-10-22 ~ 2019-11-15 LID12345 재라벨링 재검토(박정현 기사님) 반영
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled = pd.read_sql(con=con, sql='select * from tmp_jyt_relabel_ing_review_1')
        # con.close()
        # tmp_relabeled.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191028_to_19111115_lid1_2_3_4_5_reviewed.csv', index=False)

        df_relabeled_review1 = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191028_to_19111115_lid1_2_3_4_5_reviewed.csv",
            encoding="CP949",
        )

        #### 2. 2019-10-22 ~ 2019-12-06 모델 예측 안되는 데이터 선택 후 재검토(류진걸A.) 반영
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled = pd.read_sql(con=con, sql='select * from tmp_wrong_files_200102')
        # con.close()
        # tmp_relabeled.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191022_to_191206_lid1_2_3_4_5_6_7_8_reviewed1.csv', index=False)

        df_relabeled_review2 = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191022_to_191206_lid1_2_3_4_5_6_7_8_reviewed1.csv",
            encoding="CP949",
        )

        #### 3. 2019-10-22 ~ 2019-12-06 모델 예측 안되는 데이터 선택 후 재검토(류진걸A.) 반영 2차
        # con = connect_to_mysql(dBname='mc_dev')
        # tmp_relabeled = pd.read_sql(con=con, sql='select * from tmp_wrong_files_200106')
        # con.close()
        # tmp_relabeled.to_csv('/usrData1/relabel_data/jyt_relabel_list_from_191022_to_191206_lid1_2_3_4_5_6_7_8_reviewed2.csv', index=False)

        df_relabeled_review3 = pd.read_csv(
            "/usrData1/relabel_data/jyt_relabel_list_from_191022_to_191206_lid1_2_3_4_5_6_7_8_reviewed2.csv",
            encoding="CP949",
        )

        return [df_relabeled_review1, df_relabeled_review2, df_relabeled_review3]

    def check_relabel_table(self, con):
        """
        재라벨링 테이블과 Analysis_v2 테이블의 정합성 확인
        """
        data = pd.read_sql(
            con=con,
            sql=(
                "select A.FPATH, A.SR as SR_origin, B.RELABEL_METHOD, B.SR, B.SR_RELABELED from "
                "(select FPATH, SR from mc.analysis_v2 where {}) A "
                "left join "
                "(select FPATH, RELABEL_METHOD, SR, SR_RELABELED from jyt_relabel) B "
                "on A.FPATH=B.FPATH "
            ).format(self.relabeled_condition),
        )

        return data
