"""Shared Maya-style tab appearance for tool panels."""

MAYA_TAB_STYLE = """
        QTabWidget {
            background-color: #444444;
        }
        QTabWidget::pane {
            border: none;
            background-color: #444444;
        }
        QTabWidget > QStackedWidget > QWidget {
            background-color: #444444;
        }
        QTabBar {
            background-color: #373737;
            qproperty-drawBase: 0;
        }
        QTabBar::tab {
            background-color: #373737;
            padding: 2px 10px;
            min-width: 80px;
            margin-right: 2px;
            margin-top: 2px;
            border: 1px solid #2B2B2B;
            border-bottom: none;
            border-top-left-radius: 4px;
            border-top-right-radius: 4px;
        }
        QTabBar::tab:first {
            margin-left: 2px;
        }
        QTabBar::tab:selected {
            background-color: #444444;
            border: none;
        }
        QTabBar::tab:hover:!selected {
            background-color: #404040;
        }
    """
