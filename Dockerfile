# Copyright (c) Jupyter Development Team.
# Distributed under the terms of the Modified BSD License.
ARG BASE_CONTAINER=jupyter/scipy-notebook
FROM $BASE_CONTAINER

LABEL maintainer="Jupyter Project <jupyter@googlegroups.com>"

# Set when building on Travis so that certain long-running build steps can
# be skipped to shorten build time.
ARG TEST_ONLY_BUILD

USER root

# R pre-requisites
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    fonts-dejavu \
    gfortran \
    gcc && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get upgrade

#RUN apt-get install -y libncursesw5-dev
RUN apt-get install -y ncurses-base ncurses-bin
# ncurses-term

RUN apt-get update && apt-get install -y bzip2 ca-certificates automake libtool  \
                       libncurses5-dev libreadline-dev libgsl0-dev cmake ssh
WORKDIR $HOME
RUN \
  wget http://www.neuron.yale.edu/ftp/neuron/versions/v7.7/nrn-7.7.tar.gz && \
  tar -xzf nrn-7.7.tar.gz && \
  rm nrn-7.7.tar.gz

WORKDIR $HOME/nrn-7.7
ENV PATH /usr/bin/python3/python:/opt/conda/bin:/opt/conda/bin/conda:/opt/conda/bin/python:$PATH
RUN ./configure --prefix=`pwd` --without-iv --with-nrnpython=/opt/conda/bin/python3
RUN sudo make all && \
     make install

RUN make all
RUN make install

WORKDIR src/nrnpython
RUN python setup.py install
ENV NEURON_HOME $HOME/nrn-7.7/x86_64
ENV PATH $NEURON_HOME/bin:$PATH



USER root
RUN chown -R jovyan $HOME
WORKDIR $HOME/work/extra_work
WORKDIR $HOME/work


RUN git clone https://github.com/chlubba/PyPNS
WORKDIR PyPNS
RUN pip install -e .
WORKDIR mods
RUN nrnivmodl

RUN conda clean --all -f -y && \
    fix-permissions $CONDA_DIR && \
    fix-permissions /home/$NB_USER


USER $NB_UID
WORKDIR $HOME
RUN ls -ltr
RUN python -c "import PyPNS"
#CMD ["jupyter", "notebook", "--no-browser","--NotebookApp.token=''","--NotebookApp.password=''"]
#ENTRYPOINT jupyter notebook --no-browser --NotebookApp.token='' --NotebookApp.password='' 

