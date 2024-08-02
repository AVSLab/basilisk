import numpy as np
import pandas as pd


def pull_and_format_df(path, varIdxLen):
    df = pd.read_pickle(path)
    if len(np.unique(df.columns.codes[1])) is not varIdxLen:
        print("Warning: " + path + " not formatted correctly!")
        newMultIndex = pd.MultiIndex.from_product([df.columns.codes[0], list(range(varIdxLen))], names=['runNum', 'varIdx'])
        indices = pd.Index([0,1]) # Need multiple rows for curves
        df = df.reindex(columns=newMultIndex, index=indices)
    return df


def curve_per_df_component(df):
    """
    Make a curve per component in the message dataframe (i.e. omega_BR_B[2] across all runs as a single curve)

    :param df:
    :return:
    """
    idx = pd.IndexSlice
    df = df.interpolate(method = "linear")
    df_list = []
    for i in np.unique(df.columns.codes[1]):
        # Select all of the component
        varIdx_df = df.loc[idx[:], idx[:, i]]

        # Inject NaNs at the end of the run so the curves don't wrap from t_f to t_0
        varIdx_df = pd.concat([varIdx_df, pd.DataFrame([np.nan] * varIdx_df.shape[1], index=varIdx_df.columns).T])

        # Flatten values by column order
        time = np.tile(varIdx_df.index, len(varIdx_df.columns.codes[0]))  # Repeat time by number of runs
        varIdx_flat = varIdx_df.values.flatten('F')

        # Generate a curve for each component
        curve_df = pd.DataFrame(np.transpose([time, varIdx_flat]).tolist(), columns=['x', 'y'])
        df_list.append(curve_df)

    return df_list

def curve_per_df_column(df):
    """
    Divides the dataframe by column into format friendly for datashaders
    :return: List of single column dataframes
    """
    idx = pd.IndexSlice
    df_list = []
    for index in df.columns.tolist():
        try:
            i = df.columns.codes[0][index] # Multi-Index level=0 index
            j = df.columns.codes[1][index] # Multi-Index level=1 index

            # Grab the desired x and y data
            x_data = df.index.values # time [ns]
            y_data = df.loc[idx[:], idx[i, j]].values # variable data
            run_num = np.repeat(i, len(x_data))
        except:
            # Grab the desired x and y data
            x_data = df.index.values  # time [ns]
            y_data = df.loc[idx[:], idx[index]].values  # variable data
            run_num = np.repeat(index[0], len(x_data))

        # Convert to two columns
        plot_data = (pd.DataFrame(np.transpose([x_data, y_data, run_num]).tolist(), columns=['x', 'y', 'run'])
                     .values.tolist())
        df_list.append(plot_data)
    return df_list


def transform_dataframe(df_in, transforming_function):
    """
    Transforms the data in 'df_in' using the function specified in 'transforming_function'.
    E.g. if the data in 'df_in' are position vectors rvec and the 'transforming_function' is np.linalg.norm(rvec)
    then a dataframe with the norm of each position vector is returned.
    """
    num_runs = df_in.columns.levshape[0]
    num_comp = df_in.columns.levshape[1]
    num_comp_out = np.size(transforming_function(df_in.iloc[0][0].values))

    data = []
    for runNum in range(num_runs):
        result = df_in.iloc[:, num_comp * runNum:num_comp * runNum + num_comp].apply(transforming_function, axis=1, raw=True)
        if num_comp_out == 1:  # The result is one-dimensional of type Series
            data.append(result)
        else:  # The result is multi-dimensional of type dataframe
            for idx in range(num_comp_out):  # Unpack the dataframe into series
                data.append(pd.Series(result.iloc[:,idx]))
    newMultIndex = pd.MultiIndex.from_product([list(range(num_runs)), list(range(num_comp_out))], names=['runNum', 'varIdx'])
    return pd.DataFrame(data, index=newMultIndex).T


def extract_effector_df(data_path, num_eff):
    """
    Extracts the effector (RW, thruster, etc.) information for only those effectors that are used.
    E.g. the RW effector message consists by default of 36 RWs, but if only 4 are used, this function will return the
    states of only 4 RWs. The data path specifies where the data from the MC is located, for example
    '/mc_data/rw_speed_msg.wheelSpeeds.data'.
    """
    df_effector = pd.read_pickle(data_path)
    num_runs = df_effector.columns.levshape[0]
    num_eff_default = df_effector.columns.levshape[1]

    effector_list = []
    for runNum in range(num_runs):
        for effNum in range(num_eff):  # Unpack the dataframe into series
            effector_list.append(pd.Series(df_effector.iloc[:, num_eff_default * runNum + effNum]))
    newMultIndex = pd.MultiIndex.from_product([list(range(num_runs)), list(range(num_eff))], names=['runNum', 'varIdx'])
    return pd.DataFrame(effector_list, index=newMultIndex).T


def combine_dataframes(df1, df2):
    """
    Combines two dataframes by augmenting the vectors within the dataframe.
    E.g. it combines the 3x1 position vector rvec and 3x1 velocity vector vvec to one 6x1 state vector.
    By combining the dataframes, the transform_dataframe function can be used to transform multiple dataframes.
    """
    num_runs = df1.columns.levshape[0]
    num_comp1 = df1.columns.levshape[1]
    num_comp2 = df2.columns.levshape[1]
    num_comp_out = num_comp1 + num_comp2

    data = []
    for runNum in range(num_runs):
        result1 = df1.iloc[:, num_comp1 * runNum:num_comp1 * runNum + num_comp1]
        result2 = df2.iloc[:, num_comp2 * runNum:num_comp2 * runNum + num_comp2]
        result_combined = pd.concat([result1, result2], ignore_index=True, axis=1)
        for idx in range(num_comp_out):  # Unpack the dataframe into series
            data.append(pd.Series(result_combined.iloc[:, idx]))
    newMultIndex = pd.MultiIndex.from_product([list(range(num_runs)),
                                               list(range(num_comp_out))], names=['runNum', 'varIdx'])
    return pd.DataFrame(data, index=newMultIndex).T
