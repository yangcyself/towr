from distutils.core import setup, Extension

setup(name='sample',
      ext_modules=[
        Extension('sample',
                  ['sample/sample.c'],
                  include_dirs = ['../towr/include'],
                #   define_macros = [('FOO','1')],
                #   undef_macros = ['BAR'],
                  library_dirs = ['/usr/local/lib'],
                  libraries = ['sample']
                  )
        ]
)  