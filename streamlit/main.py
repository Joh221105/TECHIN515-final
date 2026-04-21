#!/usr/bin/env python3
"""
PSSS — Thermal leak viewer (Web Bluetooth to HM-10).

Requires firmware built with ENABLE_HM10_BLE=1 and HM-10 on Serial1.
Default USB dashboard: tools/ble_connect.py + web/index.html

Run:
  pip install streamlit
  streamlit run main.py
"""
from pathlib import Path

import streamlit as st
import streamlit.components.v1 as components

st.set_page_config(
    page_title="PSSS · Thermal",
    page_icon="🌡️",
    layout="wide",
    initial_sidebar_state="collapsed",
)

st.markdown("""
<style>
  #MainMenu, footer, header { visibility: hidden; height: 0; }
  .block-container { padding: 0 !important; max-width: 100% !important; }
  section[data-testid="stSidebar"] { display: none !important; }
  iframe { border: none !important; display: block !important; }
</style>
""", unsafe_allow_html=True)

components.html(
    (Path(__file__).parent / "app.html").read_text(encoding="utf-8"),
    height=960,
    scrolling=False,
)
