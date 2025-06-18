import pandas as pd

ACTUARIAL_TABLE="table.csv"
ACTUARIAL_DF = pd.read_csv(ACTUARIAL_TABLE, sep="\t",
                 names=["age-bucket", "p", "total", "num_dying", "years_lived", "years_lived_above",
                          "remaining_life"])
print(ACTUARIAL_DF)

def price_insurance(age=25, term = 10, payout = 1e7, healthiness=4, df=.20, premium=5000):
    """
    Actuarial Value a life insurance product, assuming a discount factor, fixed premium and a mulitiplicative improvement over base mortality rates.
    
    Uses CDC 2022 actuarial data from ACTUARIAL table.
    :return: 
    """

    value = 0
    for i in range(age, age + term):
        discount = (1 - df) ** (i - age)
        value -= premium * discount
        value += (1/healthiness) * discount * payout  * float(ACTUARIAL_DF["p"].iloc[i])  * float(ACTUARIAL_DF["total"].astype(float).iloc[i]) / 100_000

    return value
print(price_insurance())


