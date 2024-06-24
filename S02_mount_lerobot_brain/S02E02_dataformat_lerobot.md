# 1. Input and output of LeRobot robot brain

In [the previous article](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E01_why_to_learn_lerobot.md), we analyzed the objective of LeRobot, proposing standard APIs for robots to promote technological progress and market development. We also discussed the components included in the standard APIs, and the various technical candidate solutions for each component.

The content of this article is to look into the data format of the input and output of the LeRobot robot brain. More specifically, we will analyze the input and output data format for [the Action Chunking with Transformers (ACT)](https://link.zhihu.com/?target=https%3A//github.com/tonyzhaozh/act) robot motion planning model. 

ACT model was originally created by the Stanford Aloha team, and reprogrammed by the LeRobot team. Even though this article only analyzes the data format for ACT model, but for other models, the data format will be similar if not exactly the same. 

This article only analyzes the data format of ACT model's input and output, without dissecting the internal structure of the model. The internal structure of the model will be discussed in details in following articles.

Why shall we analyze the input and output data format of the motion planning model?

1. By deeply understanding the input and output data format of the robot brain, more technically, the robotic motion planning model, we can use Gym to implement [the environment](https://gymnasium.farama.org/tutorials/gymnasium_basics/environment_creation) for the robot brainstem. The Gym environment is the hub that connects the robot brain and body.

2. Because LeRobot is in the rapid development process, the LeRobot technical documentation is not yet completed. In order to learn LeRobot as soon as possible, we read LeRobot's source code to understand the entire system, and the first step is to examine their input and output data format.

Several issues,

1. The ACT motion planning model is just one of many robot brains. LeRobot currently has reimplemented 3 robot brains, [ACT](https://link.zhihu.com/?target=https%3A//github.com/tonyzhaozh/act), [Diffusion](https://link.zhihu.com/?target=https%3A//github.com/real-stanford/diffusion_policy), and [TD-MPC](https://link.zhihu.com/?target=https%3A//github.com/fyhMer/fowm).

    To promote the standardization of APIs for robot brains, the LeRobot team will reimplement more robotic motion planning models that are originally developed by academia and the industry, and now open-sourced. 

    This is a huge project, and even though the LeRobot team is working very hard, the technical documentation might not be completely in a short time.

2. The 3 robot brains, ACT, Diffusion, and TD-MPC, are all visual-motor models, with inputs that include videos captured by the robot's camera, but do not include voice commands from human users, etc.

    In the future, the robot brain will include more information, including not only vision but also sound, smell, taste, and touch.

    The ultimate form of the robot brain might be the Multimodal Large Language Model (MLLM). It is easy to understand why the robot brain is a multimodal model. But why the robot brain is also a language model? This question will be discussed in details in the following articles.

3.  Persuading all robot brains use standard APIs is a daunting job, but an even more difficult job is to make all robot bodies use standard APIs.

    The reason is that robot bodies are designed for different tasks, while there are commonalities in the APIs for robot bodies, there must be many differences.

    It seems that the progress of standardizing APIs for robot brains, may outpace the progress of standardizing APIs for robot bodies.

    Therefore, the data format output by the robot brain needs to be adapted to fit specific robot bodies, and the task of data conversion should be accomplished by the Gym environment, which will be detailed in the following articles.


# 2. LeRobot Training Data Stored on Huggingface

LeRobot has not only reimplemented the code for robot motion planning models such as ACT, but also converted the original training data into the data format proposed by LeRobot, which is stored on Huggingface for users to download.

![LeRobot homepage on huggingface](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E02_src/S02E02_image01_huggingface.jpeg)
![LeRobot homepage on huggingface](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E02_src/S02E02_image02_huggingface.jpeg)

For example, the ACT model has a training dataset named [aloha_sim_insertion_human](https://huggingface.co/datasets/lerobot/aloha_sim_insertion_human/tree/main). In addition to the documentation files that describe the content and format of the data, the most important file is [train-00000-of-00001.parquet](https://huggingface.co/datasets/lerobot/aloha_sim_insertion_human/tree/main/data).

We downloaded this parquet data file and then used [parquet-cli](https://github.com/chhantyal/parquet-cli) to inspect the contents of this parquet file.

~~~
$ parq train-00000-of-00001.parquet
 # Metadata
 <pyarrow._parquet.FileMetaData object at 0x120dcf470>
  created_by: parquet-cpp-arrow version 16.0.0
  num_columns: 9
  num_rows: 17000
  num_row_groups: 17
  format_version: 2.6
  serialized_size: 19910
~~~

As shown above, the file contains 17 groups of data, with each group consisting of 1,000 rows of data, totally 17,000 rows of data, and each row of data contains 9 columns.

~~~
$ parq train-00000-of-00001.parquet --schema
 # Schema
 <pyarrow._parquet.ParquetSchema object at 0x136082740>
required group field_id=-1 schema {
  optional group field_id=-1 observation.images.top {
    optional binary field_id=-1 path (String);
    optional float field_id=-1 timestamp;
  }
  optional group field_id=-1 observation.state (List) {
    repeated group field_id=-1 list {
      optional float field_id=-1 element;
    }
  }
  optional group field_id=-1 action (List) {
    repeated group field_id=-1 list {
      optional float field_id=-1 element;
    }
  }
  optional int64 field_id=-1 episode_index;
  optional int64 field_id=-1 frame_index;
  optional float field_id=-1 timestamp;
  optional boolean field_id=-1 next.done;
  optional int64 field_id=-1 index;
}
~~~

As demonstrated, each row of data includes 9 columns, which are:

1. observation.images.top["path"],
2. observation.images.top["timestamp"],
3. observation.state,
4. action,
5. episode_index,
6. frame_index,
7. timestamp,
8. next.done,
9. index.

~~~
$ parq train-00000-of-00001.parquet --head 2
                              observation.images.top  \
0  {'path': 'videos/observation.images.top_episod...
1  {'path': 'videos/observation.images.top_episod...

                                   observation.state  \
0  [0.0, -0.96, 1.16, 0.0, -0.3, 0.0, 0.0, 0.0, -...
1  [-0.0010152064, -0.9372796, 1.160248, -0.00504...

                                              action  episode_index  \
0  [-0.012271847, -0.9096506, 1.1596895, -0.00613...              0
1  [-0.012271847, -0.90658265, 1.1627575, -0.0061...              0

   frame_index  timestamp  next.done  index
0            0       0.00      False      0
1            1       0.02      False      1
~~~

As shown above, the first two rows extracted from the train-00000-of-00001.parquet file confirms that they match the schema.

Note,

The reason for choosing the [parquet](https://parquet.apache.org/docs/overview/) format for storing the files is likely because the file system used by Huggingface to store training dataset is Hadoop's HDFS. Parquet not only supports Hadoop's data processing workflow, but it also allows for reading partial data from files without the need to load the entire file into memory, making [the data reading process very efficient](https://www.upsolver.com/blog/apache-parquet-why-use).

# 3. LeRobot training dataset stored on Github

LeRobot has also stored these training datasets on Github,

![LeRobot homepage on github](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E02_src/S02E02_image03_github.jpeg)
![LeRobot homepage on github](https://github.com/housework-robot/main/blob/main/S02_mount_lerobot_brain/S02E02_src/S02E02_image04_github.jpeg)

Comparing the LeRobot training data stored on Github and Huggingface, these two datasets are not only different in their file structures , but also the file formats. On Github, the file format is arrow, e.g `data-00000-of-00001.arrow`, while on Huggingface, the file format is parquest, e.g `train-00000-of-00001.parquet`. 

The [Parquet vs Arrow](https://stackoverflow.com/questions/56472727/difference-between-apache-parquet-and-arrow) data file formats have different uses:

1. **Parquet** is used for storage; the file size is smaller. However, when used, it requires decompression and  decoding into a predetermined format for easy data reading.

2. **Arrow** is oriented towards data communication; it comes with its own data format, typically a table format composed of rows and columns. When an arrow-formatted file is opened in memory, the corresponding table appears in memory.

3. The same data can be stored as either a parquet file or an arrow file. The size of the parquet file is much smaller than that of the arrow file.

When a parquet file is opened in memory, it requires both decompression and decoding into a predetermined format, including the arrow format. Therefore, [parquet and arrow can be used in conjunction with each other](https://arrow.apache.org/cookbook/py/io.html).


# 4. Inspecting the contents of the arrow file

We wrote a Python script to inspect the contents of an Arrow file, to confirm that the contents of the arrow file match those of the parquet file.

~~~
import numpy as np
import pyarrow as pa

# arrow_file = "./data/lerobot/aloha_sim_insertion_human/train/data-00000-of-00001.arrow"
arrow_file = "data-00000-of-00001.arrow"

print(f" >> Arrow file: {arrow_file} \n\n")

with pa.memory_map(arrow_file, 'r') as source:
    loaded_arrays = pa.RecordBatchStreamReader(source).read_all()

print(">> Schema starts: ")
print(loaded_arrays.schema)
print(" << end of schema.  \n\n")

print(">> Sample data start: ")
for i in range( 8 ):
    arr = loaded_arrays[i]
    print(f"[{i}]: {arr[0]} .. {arr[-1]}")
print(" << end of sample data. ")
~~~

Here are the results of the execution:

~~~
$ python3 test_load_arrow.py
 >> Arrow file: data-00000-of-00001.arrow


>> Schema starts:
observation.images.top: struct<path: string, timestamp: float>
  child 0, path: string
  child 1, timestamp: float
observation.state: fixed_size_list<item: float>[14]
  child 0, item: float
action: fixed_size_list<item: float>[14]
  child 0, item: float
episode_index: int64
frame_index: int64
timestamp: float
next.done: bool
index: int64
-- schema metadata --
huggingface: '{"info": {"features": {"observation.images.top": {"_type": ' + 481
 << end of schema.

>> Sample data start:
[0]: [('path', 'videos/observation.images.top_episode_000000.mp4'), ('timestamp', 0.0)] .. [('path', 'videos/observation.images.top_episode_000000.mp4'), ('timestamp', 9.979999542236328)]
[1]: [0.0, -0.9599999785423279, 1.159999966621399, 0.0, -0.30000001192092896, 0.0, 0.0, 0.0, -0.9599999785423279, 1.159999966621399, 0.0, -0.30000001192092896, 0.0, 0.0] .. [-0.05793723836541176, 0.04692783206701279, 0.3877415657043457, -0.1113244965672493, 0.5908692479133606, 0.19016210734844208, 0.6397314071655273, 0.050350598990917206, -0.020691145211458206, 0.39867353439331055, -0.00458531454205513, 0.7665374875068665, 0.21321745216846466, 0.07931946963071823]
[2]: [-0.012271846644580364, -0.9096506237983704, 1.1596895456314087, -0.006135923322290182, -0.30526217818260193, 0.00920388475060463, 0.16081655025482178, 0.0015339808305725455, -0.9572040438652039, 1.1780972480773926, -0.0015339808305725455, -0.31139811873435974, 0.004601942375302315, -0.03560084104537964] .. [-0.058291271328926086, 0.04448544234037399, 0.3880971372127533, -0.11044661700725555, 0.5813787579536438, 0.19174760580062866, 0.1709628850221634, 0.05062136799097061, -0.02454369328916073, 0.3942330777645111, -0.0015339808305725455, 0.7623884677886963, 0.21322333812713623, 0.053367696702480316]
[3]: 0 .. 0
[4]: 0 .. 499
[5]: 0.0 .. 9.979999542236328
[6]: False .. True
[7]: 0 .. 499
 << end of sample data.
~~~

Comparing the contents of the arrow file with those of the parquet file, both are indeed completely identical.
