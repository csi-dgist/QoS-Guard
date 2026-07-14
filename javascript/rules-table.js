/*
 * Rule list filter.
 * Turns the static #rule-card-list into a searchable, filterable list.
 * Filters: free-text search, QoS policy, and consequence type
 * (connect = won't connect, guarantee = breaks a guarantee, resource = wastes resources).
 */
(function () {
  'use strict';

  var CONSEQUENCE_OPTIONS = [
    { value: 'connect', label: "Won't connect" },
    { value: 'guarantee', label: 'Breaks a guarantee' },
    { value: 'resource', label: 'Wastes resources' }
  ];

  function cardPolicies(card) {
    var raw = card.getAttribute('data-policies') || '';
    return raw.split('|').map(function (p) { return p.trim(); }).filter(Boolean);
  }

  function collectPolicies(cards) {
    var seen = {};
    var values = [];
    cards.forEach(function (card) {
      cardPolicies(card).forEach(function (p) {
        if (!seen[p]) { seen[p] = true; values.push(p); }
      });
    });
    return values.sort(function (a, b) { return a.localeCompare(b); });
  }

  function makeOption(value, label) {
    var o = document.createElement('option');
    o.value = value;
    o.textContent = label;
    return o;
  }

  function makeSelect(id, labelText, container) {
    var wrap = document.createElement('label');
    wrap.className = 'rules-filter-field';
    wrap.setAttribute('for', id);
    var span = document.createElement('span');
    span.className = 'rules-filter-label';
    span.textContent = labelText;
    var select = document.createElement('select');
    select.id = id;
    select.className = 'rules-filter-select';
    select.appendChild(makeOption('all', 'All'));
    wrap.appendChild(span);
    wrap.appendChild(select);
    container.appendChild(wrap);
    return select;
  }

  function initRuleFilter() {
    var list = document.getElementById('rule-card-list');
    var toolbar = document.getElementById('rule-filter-toolbar');
    if (!list || !toolbar || toolbar.getAttribute('data-ready') === '1') {
      return;
    }
    toolbar.setAttribute('data-ready', '1');

    var cards = Array.prototype.slice.call(list.querySelectorAll('.rulecard'));
    var total = cards.length;

    var bar = document.createElement('div');
    bar.className = 'rules-filter-bar';

    var searchRow = document.createElement('div');
    searchRow.className = 'rules-filter-row rules-filter-row-search';
    var searchLabel = document.createElement('label');
    searchLabel.className = 'rules-filter-field rules-filter-field-search';
    searchLabel.setAttribute('for', 'rule-filter-search');
    var searchSpan = document.createElement('span');
    searchSpan.className = 'rules-filter-label';
    searchSpan.textContent = 'Search';
    var search = document.createElement('input');
    search.type = 'search';
    search.id = 'rule-filter-search';
    search.className = 'rules-filter-input';
    search.placeholder = 'Search titles, settings, policies...';
    search.setAttribute('autocomplete', 'off');
    searchLabel.appendChild(searchSpan);
    searchLabel.appendChild(search);
    searchRow.appendChild(searchLabel);

    var selectRow = document.createElement('div');
    selectRow.className = 'rules-filter-row rules-filter-row-selects';
    var policySelect = makeSelect('rule-filter-policy', 'Policy', selectRow);
    collectPolicies(cards).forEach(function (p) {
      policySelect.appendChild(makeOption(p, p));
    });
    var consSelect = makeSelect('rule-filter-consequence', 'Consequence', selectRow);
    CONSEQUENCE_OPTIONS.forEach(function (opt) {
      consSelect.appendChild(makeOption(opt.value, opt.label));
    });

    var actionRow = document.createElement('div');
    actionRow.className = 'rules-filter-row rules-filter-row-actions';
    var reset = document.createElement('button');
    reset.type = 'button';
    reset.className = 'rules-filter-reset';
    reset.textContent = 'Reset filters';
    var count = document.createElement('p');
    count.className = 'rules-filter-count';
    count.setAttribute('aria-live', 'polite');
    actionRow.appendChild(reset);
    actionRow.appendChild(count);

    bar.appendChild(searchRow);
    bar.appendChild(selectRow);
    bar.appendChild(actionRow);
    toolbar.appendChild(bar);

    var empty = document.createElement('p');
    empty.className = 'rule-card-empty';
    empty.textContent = 'No rules match these filters.';
    empty.style.display = 'none';
    list.parentNode.insertBefore(empty, list.nextSibling);

    function matches(card) {
      var q = search.value.trim().toLowerCase();
      if (q && card.textContent.toLowerCase().indexOf(q) === -1) {
        return false;
      }
      if (policySelect.value !== 'all' && cardPolicies(card).indexOf(policySelect.value) === -1) {
        return false;
      }
      if (consSelect.value !== 'all' && card.getAttribute('data-consequence') !== consSelect.value) {
        return false;
      }
      return true;
    }

    function apply() {
      var visible = 0;
      cards.forEach(function (card) {
        var show = matches(card);
        card.style.display = show ? '' : 'none';
        if (show) { visible += 1; }
      });
      count.textContent = 'Showing ' + visible + ' of ' + total + ' rules';
      empty.style.display = visible === 0 ? '' : 'none';
    }

    function resetAll() {
      search.value = '';
      policySelect.value = 'all';
      consSelect.value = 'all';
      apply();
    }

    search.addEventListener('input', apply);
    policySelect.addEventListener('change', apply);
    consSelect.addEventListener('change', apply);
    reset.addEventListener('click', resetAll);
    apply();
  }

  function boot() {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', initRuleFilter);
    } else {
      initRuleFilter();
    }
    if (typeof document$ !== 'undefined' && document$.subscribe) {
      document$.subscribe(function () { initRuleFilter(); });
    }
  }

  boot();
})();
