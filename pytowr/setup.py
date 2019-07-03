from distutils.core import setup, Extension
import os
# import distutils
# distutils.dir_util.remove_tree()

os.environ["CC"] = "g++"
setup(name='sample',
      ext_modules=[
        Extension('pytowr',
                  ['pytowr.cc'],
                  include_dirs = ['../towr/include',"/usr/local/include", "/usr/local/include/eigen3"],
                #   define_macros = [('FOO','1')],
                #   undef_macros = ['BAR'],
                  library_dirs = ['/usr/local/lib',"/home/yangcy/programs/towr/towr/build"],
                  # libraries = ['/usr/local/lib/libifopt_ipopt.so' ,'/usr/local/lib/libifopt_core.so'],
                  libraries = ['ifopt_ipopt' ,'ifopt_core'],
                  extra_link_args = ['-rdynamic'],
                  extra_compile_args = ["-std=gnu++11"],
                  )
        ]
)  