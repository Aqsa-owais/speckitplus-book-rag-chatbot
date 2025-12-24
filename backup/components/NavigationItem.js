import React from 'react';
import clsx from 'clsx';

const NavigationItem = ({
  title,
  path,
  isActive = false,
  onClick,
  children,
  isCollapsible = false,
  isExpanded = false,
  onToggleExpand,
  className = ''
}) => {
  const handleClick = () => {
    if (onClick) {
      onClick();
    }
  };

  const handleExpandToggle = () => {
    if (onToggleExpand) {
      onToggleExpand();
    }
  };

  return (
    <li className={clsx('menu__list-item', className)}>
      <a
        href={path}
        className={clsx('menu__link', {
          'menu__link--active': isActive,
        })}
        onClick={handleClick}
      >
        {isCollapsible && (
          <button
            className="menu__caret"
            onClick={(e) => {
              e.preventDefault();
              handleExpandToggle();
            }}
            aria-label={isExpanded ? 'Collapse' : 'Expand'}
          >
            <span className={`menu__caret-icon ${isExpanded ? 'menu__caret-icon--expanded' : ''}`}>
              {isExpanded ? '▼' : '▶'}
            </span>
          </button>
        )}
        {title}
      </a>
      {isCollapsible && isExpanded && children && (
        <ul className="menu__list">{children}</ul>
      )}
    </li>
  );
};

export default NavigationItem;