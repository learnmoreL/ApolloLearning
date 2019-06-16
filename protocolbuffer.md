	Protocol Buffers 是一种轻便高效的结构化数据存储格式，可以用于序列化。可用于通讯协议、数据存储等领域的语言无关、平台无关、可扩展的序列化结构数据格式。

> ```protobuf
> package lm; 
> message helloworld 
> { 
>    required int32     id = 1;  // ID 
>    required string    str = 2;  // str 
>    optional int32     opt = 3;  //optional field 
> }
> ```

​	写好lm.helloworld.proto文件，定义我们程序中需要处理的结构化数据Message。

> ```protobuf
> package lm; 
> message helloworld 
> { 
>    required int32     id = 1;  // ID 
>    required string    str = 2;  // str 
>    optional int32     opt = 3;  //optional field 
> }
> ```

用 Protobuf 编译器将该文件编译成目标语言。proto 文件存放在 $SRC_DIR 下面，把生成的文件放在同一个目录下，则可以使用如下命令：

```cmake
`protoc -I=$SRC_DIR --cpp_out=$DST_DIR $SRC_DIR/addressbook.proto`
```

命令将生成两个文件：

lm.helloworld.pb.h ， 定义了 C++ 类的头文件

lm.helloworld.pb.cc ， C++ 类的实现文件



在 Writer 代码中，将要存入磁盘的结构化数据由一个 lm::helloworld 类的对象表示，它提供了一系列的 get/set 函数用来修改和读取结构化数据中的数据成员，或者叫 field。当我们需要将该结构化数据保存到磁盘上时，类 lm::helloworld 已经提供相应的方法来把一个复杂的数据变成一个字节序列，我们可以将这个字节序列写入磁盘。SerializeToOstream 将对象序列化后写入一个 fstream 流。

> ```protobuf
>  #include "lm.helloworld.pb.h"
> …
>  int main(void) 
>  { 
>   
>   lm::helloworld msg1; 
>   msg1.set_id(101); 
>   msg1.set_str(“hello”); 
>     
>   ``// Write the new address book back to disk. 
>   fstream output("./log", ios::out | ios::trunc | ios::binary); 
>         
>   if (!msg1.SerializeToOstream(&output)) { 
>       cerr << "Failed to write msg." << endl; 
>       return -1; 
>   }         
>   return 0; 
>  }
> ```

同样，Reader 声明类 helloworld 的对象 msg1，然后利用 ParseFromIstream 从一个 fstream 流中读取信息并反序列化。此后，ListMsg 中采用 get 方法读取消息的内部信息，并进行打印输出操作。

> ```protobuf
> #include "lm.helloworld.pb.h" 
> …
> void ListMsg(const lm::helloworld & msg) { 
> cout << msg.id() << endl; 
> cout << msg.str() << endl; 
> } 
> 
> int main(int argc, char* argv[]) { 
> 	lm::helloworld msg1; 
> 	{ 
>      fstream input("./log", ios::in | ios::binary); 
>      if (!msg1.ParseFromIstream(&input)) { 
>        cerr << "Failed to parse address book." << endl; 
>        return -1; 
>  	} 
> } 
> 
> ListMsg(msg1); 
> … 
> }
> ```