# ShapeMatch
Edge based Template Matching
学习的过程众发现一篇文章 https://www.codeproject.com/articles/99457/edge-based-template-matching
作为方案验证很好，所以计划把它写成一个离实际应用更广一点的应用，于是开启这个项目
边缘提取部分依旧沿用作者的canny算子，需要加入滤波操作
模板文件根据工程化应用设置更完善的结构，目的是可以保存文件，读取文件
加入旋转模板的策略，建立模板序列，尽量使用C语言完成
