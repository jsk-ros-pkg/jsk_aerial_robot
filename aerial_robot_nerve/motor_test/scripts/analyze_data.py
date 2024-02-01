import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def analyze_data(folder_path, file_name, has_telemetry, set_voltage):
    file_path = folder_path + file_name
    data = pd.read_csv(file_path, sep=' ',
                       names=["PWM", "fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature",
                              "voltage", "State"])

    # Filter out the rows where State is 'done'
    data = data[data['PWM'] != 'done']

    # convert PWM from string to int
    data['PWM'] = data['PWM'].astype(int)

    # Filter the DataFrame for 'valid' state
    valid_data = data[data['State'] == 'valid']

    if not has_telemetry:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency"]
    else:
        columns_to_average = ["fx", "fy", "fz", "f_norm", "mx", "my", "mz", "currency", "RPM", "temperature", "voltage"]

    # Group by 'PWM' and calculate the average for each specified column
    average_values = valid_data.groupby('PWM')[columns_to_average].mean()

    # Resetting index to make 'PWM' a column again
    average_values.reset_index(inplace=True)

    # Add a column for power
    if has_telemetry:
        average_values['Power'] = average_values['voltage'] * average_values['currency']
    else:
        average_values['Power'] = set_voltage * average_values['currency']

    # Add a column for PWM ratio
    average_values['PWM_Ratio_%'] = average_values['PWM'] / 2000 * 100

    if has_telemetry:
        # Add a column for kRPM^2 (kilorevolutions per minute squared)
        average_values['kRPM^2'] = (average_values['RPM'] / 1000) ** 2
        # Add a column for Dshot ratio in percentage
        average_values['Dshot_Ratio_%'] = ((average_values['PWM'] - 1000) / 1000) * 100

    ''' plot the data '''

    # Convert series to numpy arrays

    if has_telemetry:
        kRPM2 = average_values['kRPM^2'].to_numpy()

    fz = average_values['fz'].to_numpy()
    mz = average_values['mz'].to_numpy()
    currency = average_values['currency'].to_numpy()
    PWM_ratio = average_values['PWM_Ratio_%'].to_numpy()

    # Perform the fittings
    if has_telemetry:
        # Linear fit for x:kRPM^2 y:fz
        slope_kRPM2, intercept_kRPM2 = np.polyfit(kRPM2, fz, 1)
        fit_eq_kRPM2 = f"fz = {slope_kRPM2:.4f} * kRPM^2 + {intercept_kRPM2:.4f}"
        print(f"Fitting Equation (x:kRPM^2 y:fz): {fit_eq_kRPM2}")

    # Linear fit for x:fz y:mz, note that the intercept must be 0. Only y=kx form.
    slope_mz_fz, intercept_mz_fz = np.polyfit(fz, mz, 1)
    fit_eq_mz_fz = f"mz = {slope_mz_fz:.4f} * fz"
    print(f"Fitting Equation (x:fz y:mz): {fit_eq_mz_fz}")

    # 2nd-order polynomial fit for x:fz y:currency
    coeffs_currency_fz = np.polyfit(fz, currency, 2)
    fit_eq_currency_fz = f"currency = {coeffs_currency_fz[0]:.4e} * fz^2 + {coeffs_currency_fz[1]:.4e} * fz + {coeffs_currency_fz[2]:.4f}"
    print(f"Fitting Equation (x:fz y:currency): {fit_eq_currency_fz}")

    # 2nd-order polynomial fit for x:PWM_ratio y:fz
    coeffs_PWM_ratio_fz = np.polyfit(PWM_ratio, fz, 2)
    fit_eq_PWM_ratio_fz = f"fz = {coeffs_PWM_ratio_fz[0]:.4e} * PWM_Ratio_%^2 + {coeffs_PWM_ratio_fz[1]:.4e} * PWM_Ratio_% + {coeffs_PWM_ratio_fz[2]:.4f}"
    print(f"Fitting Equation (x:PWM_Ratio_% y:fz): {fit_eq_PWM_ratio_fz}")
    print(f"max fz: {np.max(fz):.4f} N")
    print(f"polynominal2(10x): {10 * coeffs_PWM_ratio_fz[0]:.5f}")
    print(f"polynominal1(10x): {10 * coeffs_PWM_ratio_fz[1]:.5f}")
    print(f"polynominal0: {coeffs_PWM_ratio_fz[2]:.5f}")

    # Create 2x2 subplots
    fig, axs = plt.subplots(2, 2, figsize=(12, 12))

    # Plot 1: x:kRPM^2 y:fz
    if has_telemetry:
        print("has telemetry", has_telemetry)
        axs[0, 0].scatter(kRPM2, fz, color='blue', alpha=0.5, label='Data points')
        axs[0, 0].plot(kRPM2, slope_kRPM2 * kRPM2 + intercept_kRPM2, color='red', label='Fitted line')
        axs[0, 0].set_title('fz vs kRPM^2')
        axs[0, 0].set_xlabel('kRPM^2')
        axs[0, 0].set_ylabel('fz (N)')
        axs[0, 0].grid()
        axs[0, 0].text(0.95, 0.05, fit_eq_kRPM2, horizontalalignment='right', verticalalignment='bottom',
                       transform=axs[0, 0].transAxes, fontsize=10, color='green')

    # Plot 2: x:fz y:mz
    axs[0, 1].scatter(fz, mz, color='blue', alpha=0.5, label='Data points')
    axs[0, 1].plot(fz, slope_mz_fz * fz, color='red', label='Fitted line')
    axs[0, 1].set_title('mz vs fz')
    axs[0, 1].set_xlabel('fz (N)')
    axs[0, 1].set_ylabel('mz (N*m)')
    axs[0, 1].grid()
    axs[0, 1].text(0.95, 0.05, fit_eq_mz_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[0, 1].transAxes, fontsize=10, color='green')

    # Plot 3: x:fz y:currency
    axs[1, 0].scatter(fz, currency, color='blue', alpha=0.5, label='Data points')
    axs[1, 0].plot(np.sort(fz), np.polyval(coeffs_currency_fz, np.sort(fz)), color='red', label='Fitted polynomial')
    axs[1, 0].set_title('currency vs fz')
    axs[1, 0].set_xlabel('fz (N)')
    axs[1, 0].set_ylabel('currency')
    axs[1, 0].grid()
    axs[1, 0].text(0.95, 0.05, fit_eq_currency_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[1, 0].transAxes, fontsize=10, color='green')

    # Plot 4: fz vs PWM_ratio
    axs[1, 1].scatter(PWM_ratio, fz, color='blue', alpha=0.5, label='Data points')
    axs[1, 1].plot(np.sort(PWM_ratio), np.polyval(coeffs_PWM_ratio_fz, np.sort(PWM_ratio)), color='red',
                   label='Fitted polynomial')
    axs[1, 1].set_title('fz vs PWM_Ratio_%')
    axs[1, 1].set_xlabel('PWM_Ratio_%')
    axs[1, 1].set_ylabel('fz (N)')
    axs[1, 1].grid()
    axs[1, 1].text(0.95, 0.05, fit_eq_PWM_ratio_fz, horizontalalignment='right', verticalalignment='bottom',
                   transform=axs[1, 1].transAxes, fontsize=10, color='green')

    # title with the file name
    fig.suptitle(file_name)

    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    # pass text file name as argument
    parser = argparse.ArgumentParser()
    parser.add_argument('file_name', help='file name')
    parser.add_argument('has_telemetry', help='has telemetry, 0 or 1')
    parser.add_argument("--set_voltage", help="the voltage set in the test, float", default=0.0)
    parser.add_argument('--folder_path', help='path to folder', default='~/.ros/')
    args = parser.parse_args()

    analyze_data(args.folder_path, args.file_name, int(args.has_telemetry), float(args.set_voltage))
