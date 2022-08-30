#!/usr/bin/env bash

# Kernel
git pull msm-5.4 $1

# Techpacks
git subtree pull --prefix=techpack/audio audio $1
git subtree pull --prefix=techpack/camera camera $1
git subtree pull --prefix=techpack/dataipa dataipa $1
git subtree pull --prefix=techpack/datarmnet datarmnet $1
git subtree pull --prefix=techpack/datarmnet-ext datarmnet-ext $1
git subtree pull --prefix=techpack/display display $1
git subtree pull --prefix=techpack/video video $1

# Wi-Fi
git subtree pull --prefix=drivers/staging/fw-api fw-api $1
git subtree pull --prefix=drivers/staging/qca-wifi-host-cmn qca-wifi-host-cmn $1
git subtree pull --prefix=drivers/staging/qcacld-3.0 qcacld-3.0 $1
