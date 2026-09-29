/* SPDX-License-Identifier: ISC */
/*
 * Copyright (c) 2015,2017 Qualcomm Atheros, Inc.
 */
#ifndef _WOW_H_
#define _WOW_H_

struct ath10k_wow {
	u32 max_num_patterns;
	struct completion wakeup_completed;
	struct wiphy_wowlan_support wowlan_support;
};

#ifdef CONFIG_PM

int ath10k_wow_init(struct ath10k *ar);
int ath10k_wow_op_suspend(struct ieee80211_hw *hw,
			  struct cfg80211_wowlan *wowlan);
int ath10k_wow_op_resume(struct ieee80211_hw *hw);
void ath10k_wow_op_set_wakeup(struct ieee80211_hw *hw, bool enabled);
int ath10k_sched_scan_start(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
			    struct cfg80211_sched_scan_request *req,
			    struct ieee80211_scan_ies *ies);
int ath10k_sched_scan_stop(struct ieee80211_hw *hw, struct ieee80211_vif *vif);

#else

static inline int ath10k_wow_init(struct ath10k *ar)
{
	return 0;
}

#endif /* CONFIG_PM */
#endif /* _WOW_H_ */
