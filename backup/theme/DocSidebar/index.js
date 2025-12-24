import React, { useEffect, useState } from 'react';
import clsx from 'clsx';
import {
  useThemeConfig,
} from '@docusaurus/theme-common';
import {
  splitOrThrow,
  useLocalPathname,
  useAnnouncementBar,
  useScrollPosition,
} from '@docusaurus/theme-common/internal';
import { translate } from '@docusaurus/Translate';
import DocSidebarItems from '@theme/DocSidebarItems';
import { useTabContext } from '../../context/TabContext';

// Function to determine which module a document belongs to
const getModuleForDoc = (docPath) => {
  if (docPath.includes('/module-1/') || docPath.includes('module1')) {
    return 'module1';
  } else if (docPath.includes('/module-2/') || docPath.includes('module2')) {
    return 'module2';
  } else if (docPath.includes('/module-3/') || docPath.includes('module3')) {
    return 'module3';
  } else if (docPath.includes('/module-4/') || docPath.includes('module4')) {
    return 'module4';
  }
  return null; // Home or other docs that don't belong to a specific module
};

// Function to filter sidebar items based on active module
const filterSidebarItems = (items, activeTab) => {
  if (activeTab === 'home') {
    // For home tab, show all items
    return items;
  } else {
    // For module tabs, only show items belonging to the active module
    return items.filter(item => {
      if (item.type === 'category') {
        // For categories, check if any of their children belong to the module
        const filteredChildren = filterSidebarItems(item.items, activeTab);
        return filteredChildren.length > 0;
      } else if (item.type === 'doc') {
        // For documents, check if they belong to the active module
        return getModuleForDoc(item.docPath || item.href) === activeTab;
      }
      return true; // For other types, keep them
    }).map(item => {
      // If it's a category, also filter its children
      if (item.type === 'category') {
        return {
          ...item,
          items: filterSidebarItems(item.items, activeTab)
        };
      }
      return item;
    });
  }
};

const DocSidebar = (props) => {
  const { activeTab } = useTabContext().state;
  const [filteredItems, setFilteredItems] = useState(props.items);
  const [showResponsiveSidebar, setShowResponsiveSidebar] = useState(false);

  const {
    sidebar: { items: sidebarItems, className },
  } = props;

  const themeConfig = useThemeConfig();
  const { navbar: { hideOnScroll } = {}, sidebar: { hideable = false } = {} } = themeConfig;
  const { isAnnouncementBarClosed } = useAnnouncementBar();
  const [hiddenSidebar, setHiddenSidebar] = useState(false);
  const [scrollY, setScrollY] = useState(0);

  const pathname = useLocalPathname();

  useScrollPosition(
    ({ scrollY }) => {
      setScrollY(scrollY);
    },
    [pathname],
  );

  // Filter sidebar items based on active tab
  useEffect(() => {
    const filtered = filterSidebarItems(sidebarItems, activeTab);
    setFilteredItems(filtered);
  }, [sidebarItems, activeTab]);

  const responsiveSidebarClassNames = clsx('menu', 'menu--responsive', {
    'menu--show': showResponsiveSidebar,
  });

  return (
    <aside
      className={clsx(
        'col',
        {
          'col--3': !hiddenSidebar,
          'col--0': hiddenSidebar,
          'menu__responsive-close': !showResponsiveSidebar,
        },
        className,
      )}
      onTransitionEnd={(e) => {
        if (e.propertyName === 'margin' && hiddenSidebar) {
          document.body.classList.add('docSidebarHidden');
        }
      }}
    >
      {hideable && (
        <button
          aria-label={hiddenSidebar ? translate({ id: 'theme.docs.sidebar.expand' }) : translate({ id: 'theme.docs.sidebar.collapse' })}
          className="button button--secondary button--outline button--sm margin-bottom--md doc-sidebar__toggle"
          type="button"
          onClick={() => {
            setHiddenSidebar(!hiddenSidebar);
            if (!hiddenSidebar) {
              document.body.classList.add('docSidebarHidden');
            } else {
              document.body.classList.remove('docSidebarHidden');
            }
          }}
        >
          <svg className="icon--md" viewBox="0 0 24 24">
            <path d="M3 18h18v-2H3v2zm0-5h18v-2H3v2zm0-7v2h18V6H3z" />
          </svg>
        </button>
      )}

      <div
        className={responsiveSidebarClassNames}
        onTransitionEnd={(e) => {
          if (e.propertyName === 'opacity' && !showResponsiveSidebar) {
            document.body.classList.remove('overflowHidden');
          }
          if (e.propertyName === 'transform' && showResponsiveSidebar) {
            document.body.classList.add('overflowHidden');
          }
        }}
      >
        <button
          aria-label={translate({ id: 'theme.docs.sidebar.close' })}
          className="button button--secondary button--outline menu__caret"
          type="button"
          onClick={() => {
            setShowResponsiveSidebar(false);
          }}
        >
          <svg className="icon--sm" viewBox="0 0 24 24">
            <path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z" />
          </svg>
        </button>

        <ul className="menu__list">
          <DocSidebarItems
            items={filteredItems}
            activeDocSlug={props.activeDocSlug}
            collapsed={props.collapsed}
            isChild={false}
          />
        </ul>
      </div>
    </aside>
  );
};

export default DocSidebar;