
To launch monogdb visualizatoin tool:
`mongodb-compass --no-sandbox`

we are using the `--no-sandbox` because if this error

`
Failed to move to new namespace: PID namespaces supported, Network namespace supported, but failed: errno = Operation not permitted
[14994:0525/132430.117654:FATAL:zygote_host_impl_linux.cc(202)] Check failed: . : Invalid argument (22)
Trace/breakpoint trap (core dumped)
`

Run MongoDB Compass without Sandbox: you can try running MongoDB Compass without the sandbox feature. This is not ideal for security reasons but might help bypass the immediate problem. You can try launching MongoDB Compass with the `--no-sandbox` flag. 

