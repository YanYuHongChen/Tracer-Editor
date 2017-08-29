Tracer Material Editor
======

# Summary

It is a material editor and previewer for my "Tracer" physically based renderer which is based on [dear imgui](https://github.com/ocornut/imgui).
The scene file make up with an obj file which stores geometry data and a .sce file which stores material data.

"Tracer" is my first physically based renderer but I am not ready to share it because the suffering from so many drawbacks. I am now rewriting it for a better architecture(try to implement the architecture which is mentioned in [1]), performance and more production features which may be make it more production like.

However the editor code is independent of the renderer which makes it may be used in other code.

# Editor 

![](https://github.com/wubugui/FXXKTracer/raw/master/pic/editor1.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/editor.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/meditor1.png)

# "Tracer" Gallery

![](https://github.com/wubugui/FXXKTracer/raw/master/pic/mat1.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/shelf.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/ocean_166666.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/_grass2.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/_ball.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/_nuonv.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/_shinei.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/girl111.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/silk.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/long2.png)
![](https://github.com/wubugui/FXXKTracer/raw/master/pic/newpic/disney.png)


# Reference

[1]Eisenacher C, Nichols G, Selle A, et al. Sorted Deferred Shading for Production Path Tracing[J]. Computer Graphics Forum, 2013, 32(4):125–132.