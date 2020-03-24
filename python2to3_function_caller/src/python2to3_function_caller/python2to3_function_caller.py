#!/usr/bin/env python

import execnet

class python2to3FunctionCaller():
    def call_python_version(self, Version, Module, Function, ArgumentList):
        gw      = execnet.makegateway("popen//python=python%s" % Version)
        channel = gw.remote_exec("""
            from %s import %s as the_function
            channel.send(the_function(*channel.receive()))
        """ % (Module, Function))
        channel.send(ArgumentList)
        return channel.receive()

