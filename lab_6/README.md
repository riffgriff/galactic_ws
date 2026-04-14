## Lab 6 (Vision Checkpoint)

1. Create and activate environment

`conda create -n lab6_env python=3.12 -y`

`conda activate lab6_env`

2. Install dependencies from Lab 6 requirements

`pip install -r lab_6/requirements.txt`

3. Train augmented KNN model

`python lab_6/knn.py -p lab_6/2026S_imgs/ -r 0.9 -k 7 -s -n lab_6/model -t`

4. Evaluate with grader using XML model

`python lab_6/model_grader.py --data_path lab_6/2026S_imgs --model_path lab_6/model.xml`